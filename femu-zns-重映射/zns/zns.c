#include "./zns.h"

#define MIN_DISCARD_GRANULARITY     (4 * KiB)
#define NVME_DEFAULT_ZONE_SIZE      (128 * MiB)
#define NVME_DEFAULT_MAX_AZ_SIZE    (128 * KiB)

// //4g默认一个lun是32个blks
#define BLKS_PER_LUN 32*8
#define BLKS_PER_CH 32*8*4
#define BLKS_PER_ZONE 32
#define ERASE_THRESHOLD 32
#define REMAP_BETWEEN_ZONES 96
static void zns_zone_blks_remaping(FemuCtrl *n, NvmeZone *zone,uint32_t zone_id);

static inline uint32_t zns_zone_idx(NvmeNamespace *ns, uint64_t slba)
{
    FemuCtrl *n = ns->ctrl;
    
    return (n->zone_size_log2 > 0 ? slba >> n->zone_size_log2 : slba / n->zone_size);
}

static inline NvmeZone *zns_get_zone_by_slba(NvmeNamespace *ns, uint64_t slba)
{
    FemuCtrl *n = ns->ctrl;
    uint32_t zone_idx = zns_zone_idx(ns, slba);
    // femu_debug("zone_idx=%u\n\r",zone_idx);
    
    assert(zone_idx < n->num_zones);
    return &n->zone_array[zone_idx];
}

static int zns_init_zone_geometry(NvmeNamespace *ns, Error **errp)
{
    FemuCtrl *n = ns->ctrl;
    uint64_t zone_size, zone_cap;
    uint32_t lbasz = 1 << zns_ns_lbads(ns);

    if (n->zone_size_bs) {
        zone_size = n->zone_size_bs;
    } else {
        zone_size = NVME_DEFAULT_ZONE_SIZE;
    }

    if (n->zone_cap_bs) {
        zone_cap = n->zone_cap_bs;
    } else {
        zone_cap = zone_size;
    }

    if (zone_cap > zone_size) {
        femu_err("zone capacity %luB > zone size %luB", zone_cap, zone_size);
        return -1;
    }
    if (zone_size < lbasz) {
        femu_err("zone size %luB too small, must >= %uB", zone_size, lbasz);
        return -1;
    }
    if (zone_cap < lbasz) {
        femu_err("zone capacity %luB too small, must >= %uB", zone_cap, lbasz);
        return -1;
    }

    n->zone_size = zone_size / lbasz;
    n->zone_capacity = zone_cap / lbasz;
    n->num_zones = ns->size / lbasz / n->zone_size;
    femu_debug("Zone: num_zones = %u, lbasz = %u, ns_size=%ld\n\r", n->num_zones, lbasz, ns->size);

    if (n->max_open_zones > n->num_zones) {
        femu_err("max_open_zones value %u exceeds the number of zones %u",
                 n->max_open_zones, n->num_zones);
        return -1;
    }
    if (n->max_active_zones > n->num_zones) {
        femu_err("max_active_zones value %u exceeds the number of zones %u",
                 n->max_active_zones, n->num_zones);
        return -1;
    }

    if (n->zd_extension_size) {
        if (n->zd_extension_size & 0x3f) {
            femu_err("zone descriptor extension size must be multiples of 64B");
            return -1;
        }
        if ((n->zd_extension_size >> 6) > 0xff) {
            femu_err("zone descriptor extension size is too large");
            return -1;
        }
    }

    return 0;
}

static void zns_init_zoned_state(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    uint64_t start = 0, zone_size = n->zone_size;
    uint64_t capacity = n->num_zones * zone_size;
    NvmeZone *zone;
    int i;

    n->zone_array = g_new0(NvmeZone, n->num_zones);
    if (n->zd_extension_size) {
        n->zd_extensions = g_malloc0(n->zd_extension_size * n->num_zones);
    }

    QTAILQ_INIT(&n->exp_open_zones);
    QTAILQ_INIT(&n->imp_open_zones);
    QTAILQ_INIT(&n->closed_zones);
    QTAILQ_INIT(&n->full_zones);

    zone = n->zone_array;
    for (i = 0; i < n->num_zones; i++, zone++) {
        if (start + zone_size > capacity) {
            zone_size = capacity - start;
        }
        zone->d.zt = NVME_ZONE_TYPE_SEQ_WRITE;
        zns_set_zone_state(zone, NVME_ZONE_STATE_EMPTY);
        zone->d.za = 0;
        zone->d.zcap = n->zone_capacity;
        zone->d.zslba = start;
        zone->d.wp = start;
        zone->w_ptr = start;
        start += zone_size;
        //addition
        zone->blks = (struct zns_blk**)g_malloc0(sizeof(struct zns_blk*) * BLKS_PER_ZONE);
        zone->erase_count = 0;
    }

    n->zone_size_log2 = 0;
    if (is_power_of_2(n->zone_size)) {
        n->zone_size_log2 = 63 - clz64(n->zone_size);
    }
}

static void zns_init_zone_identify(FemuCtrl *n, NvmeNamespace *ns, int lba_index)
{
    NvmeIdNsZoned *id_ns_z;

    zns_init_zoned_state(ns);

    id_ns_z = g_malloc0(sizeof(NvmeIdNsZoned));

    /* MAR/MOR are zeroes-based, 0xffffffff means no limit */
    id_ns_z->mar = cpu_to_le32(n->max_active_zones - 1);
    id_ns_z->mor = cpu_to_le32(n->max_open_zones - 1);
    id_ns_z->zoc = 0;
    id_ns_z->ozcs = n->cross_zone_read ? 0x01 : 0x00;

    id_ns_z->lbafe[lba_index].zsze = cpu_to_le64(n->zone_size);
    id_ns_z->lbafe[lba_index].zdes = n->zd_extension_size >> 6; /* Units of 64B */

    n->csi = NVME_CSI_ZONED;
    ns->id_ns.nsze = cpu_to_le64(n->num_zones * n->zone_size);
    ns->id_ns.ncap = ns->id_ns.nsze;
    ns->id_ns.nuse = ns->id_ns.ncap;

    /* NvmeIdNs */
    /*
     * The device uses the BDRV_BLOCK_ZERO flag to determine the "deallocated"
     * status of logical blocks. Since the spec defines that logical blocks
     * SHALL be deallocated when then zone is in the Empty or Offline states,
     * we can only support DULBE if the zone size is a multiple of the
     * calculated NPDG.
     */
    if (n->zone_size % (ns->id_ns.npdg + 1)) {
        femu_err("the zone size (%"PRIu64" blocks) is not a multiple of the"
                 "calculated deallocation granularity (%"PRIu16" blocks); DULBE"
                 "support disabled", n->zone_size, ns->id_ns.npdg + 1);
        ns->id_ns.nsfeat &= ~0x4;
    }

    n->id_ns_zoned = id_ns_z;
}

static void zns_clear_zone(NvmeNamespace *ns, NvmeZone *zone)
{
    FemuCtrl *n = ns->ctrl;
    uint8_t state;

    zone->w_ptr = zone->d.wp;
    state = zns_get_zone_state(zone);
    if (zone->d.wp != zone->d.zslba || (zone->d.za & NVME_ZA_ZD_EXT_VALID)) {
        if (state != NVME_ZONE_STATE_CLOSED) {
            zns_set_zone_state(zone, NVME_ZONE_STATE_CLOSED);
        }
        zns_aor_inc_active(ns);
        QTAILQ_INSERT_HEAD(&n->closed_zones, zone, entry);
    } else {
        zns_set_zone_state(zone, NVME_ZONE_STATE_EMPTY);
    }
}

static void zns_zoned_ns_shutdown(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    NvmeZone *zone, *next;

    QTAILQ_FOREACH_SAFE(zone, &n->closed_zones, entry, next) {
        QTAILQ_REMOVE(&n->closed_zones, zone, entry);
        zns_aor_dec_active(ns);
        zns_clear_zone(ns, zone);
    }
    QTAILQ_FOREACH_SAFE(zone, &n->imp_open_zones, entry, next) {
        QTAILQ_REMOVE(&n->imp_open_zones, zone, entry);
        zns_aor_dec_open(ns);
        zns_aor_dec_active(ns);
        zns_clear_zone(ns, zone);
    }
    QTAILQ_FOREACH_SAFE(zone, &n->exp_open_zones, entry, next) {
        QTAILQ_REMOVE(&n->exp_open_zones, zone, entry);
        zns_aor_dec_open(ns);
        zns_aor_dec_active(ns);
        zns_clear_zone(ns, zone);
    }

    assert(n->nr_open_zones == 0);
}

void zns_ns_shutdown(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    if (n->zoned) {
        zns_zoned_ns_shutdown(ns);
    }
}

void zns_ns_cleanup(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    if (n->zoned) {
        g_free(n->id_ns_zoned);
        g_free(n->zone_array);
        g_free(n->zd_extensions);
    }
}

static void zns_assign_zone_state(NvmeNamespace *ns, NvmeZone *zone, NvmeZoneState state)
{
    FemuCtrl *n = ns->ctrl;

    if (QTAILQ_IN_USE(zone, entry)) {
        switch (zns_get_zone_state(zone)) {
        case NVME_ZONE_STATE_EXPLICITLY_OPEN:
            QTAILQ_REMOVE(&n->exp_open_zones, zone, entry);
            break;
        case NVME_ZONE_STATE_IMPLICITLY_OPEN:
            QTAILQ_REMOVE(&n->imp_open_zones, zone, entry);
            break;
        case NVME_ZONE_STATE_CLOSED:
            QTAILQ_REMOVE(&n->closed_zones, zone, entry);
            break;
        case NVME_ZONE_STATE_FULL:
            QTAILQ_REMOVE(&n->full_zones, zone, entry);
        default:
            ;
        }
    }

    zns_set_zone_state(zone, state);

    switch (state) {
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        QTAILQ_INSERT_TAIL(&n->exp_open_zones, zone, entry);
        break;
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        QTAILQ_INSERT_TAIL(&n->imp_open_zones, zone, entry);
        break;
    case NVME_ZONE_STATE_CLOSED:
        QTAILQ_INSERT_TAIL(&n->closed_zones, zone, entry);
        break;
    case NVME_ZONE_STATE_FULL:
        QTAILQ_INSERT_TAIL(&n->full_zones, zone, entry);
    case NVME_ZONE_STATE_READ_ONLY:
        break;
    default:
        zone->d.za = 0;
    }
}

/*
 * Check if we can open a zone without exceeding open/active limits.
 * AOR stands for "Active and Open Resources" (see TP 4053 section 2.5).
 */
static int zns_aor_check(NvmeNamespace *ns, uint32_t act, uint32_t opn)
{
    FemuCtrl *n = ns->ctrl;
    if (n->max_active_zones != 0 &&
        n->nr_active_zones + act > n->max_active_zones) {
        return NVME_ZONE_TOO_MANY_ACTIVE | NVME_DNR;
    }
    if (n->max_open_zones != 0 &&
        n->nr_open_zones + opn > n->max_open_zones) {
        return NVME_ZONE_TOO_MANY_OPEN | NVME_DNR;
    }

    return NVME_SUCCESS;
}

static uint16_t zns_check_zone_state_for_write(NvmeZone *zone)
{
    uint16_t status;

    switch (zns_get_zone_state(zone)) {
    case NVME_ZONE_STATE_EMPTY:
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
    case NVME_ZONE_STATE_CLOSED:
        status = NVME_SUCCESS;
        break;
    case NVME_ZONE_STATE_FULL:
        status = NVME_ZONE_FULL;
        break;
    case NVME_ZONE_STATE_OFFLINE:
        status = NVME_ZONE_OFFLINE;
        break;
    case NVME_ZONE_STATE_READ_ONLY:
        status = NVME_ZONE_READ_ONLY;
        break;
    default:
        assert(false);
    }

    return status;
}

static uint16_t zns_check_zone_write(FemuCtrl *n, NvmeNamespace *ns,
                                     NvmeZone *zone, uint64_t slba,
                                     uint32_t nlb, bool append)
{
    uint16_t status;

    if (unlikely((slba + nlb) > zns_zone_wr_boundary(zone))) {
        status = NVME_ZONE_BOUNDARY_ERROR;
    } else {
        status = zns_check_zone_state_for_write(zone);
    }

    if (status != NVME_SUCCESS) {
    } else {
        assert(zns_wp_is_valid(zone));
        if (append) {
            if (unlikely(slba != zone->d.zslba)) {
                status = NVME_INVALID_FIELD;
            }
            if (zns_l2b(ns, nlb) > (n->page_size << n->zasl)) {
                status = NVME_INVALID_FIELD;
            }
        } else if (unlikely(slba != zone->w_ptr)) {
            status = NVME_ZONE_INVALID_WRITE;
        }
    }

    return status;
}

static uint16_t zns_check_zone_state_for_read(NvmeZone *zone)
{
    uint16_t status;

    switch (zns_get_zone_state(zone)) {
    case NVME_ZONE_STATE_EMPTY:
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
    case NVME_ZONE_STATE_FULL:
    case NVME_ZONE_STATE_CLOSED:
    case NVME_ZONE_STATE_READ_ONLY:
        status = NVME_SUCCESS;
        break;
    case NVME_ZONE_STATE_OFFLINE:
        status = NVME_ZONE_OFFLINE;
        break;
    default:
        assert(false);
    }

    return status;
}

static uint16_t zns_check_zone_read(NvmeNamespace *ns, uint64_t slba, uint32_t nlb)
{
    FemuCtrl *n = ns->ctrl;
    NvmeZone *zone = zns_get_zone_by_slba(ns, slba);
    uint64_t bndry = zns_zone_rd_boundary(ns, zone);
    uint64_t end = slba + nlb;
    uint16_t status;

    status = zns_check_zone_state_for_read(zone);
    if (status != NVME_SUCCESS) {
        ;
    } else if (unlikely(end > bndry)) {
        if (!n->cross_zone_read) {
            status = NVME_ZONE_BOUNDARY_ERROR;
        } else {
            /*
             * Read across zone boundary - check that all subsequent
             * zones that are being read have an appropriate state.
             */
            do {
                zone++;
                status = zns_check_zone_state_for_read(zone);
                if (status != NVME_SUCCESS) {
                    break;
                }
            } while (end > zns_zone_rd_boundary(ns, zone));
        }
    }

    return status;
}

static void zns_auto_transition_zone(NvmeNamespace *ns)
{
    FemuCtrl *n = ns->ctrl;
    NvmeZone *zone;

    if (n->max_open_zones &&
        n->nr_open_zones == n->max_open_zones) {
        zone = QTAILQ_FIRST(&n->imp_open_zones);
        if (zone) {
             /* Automatically close this implicitly open zone */
            QTAILQ_REMOVE(&n->imp_open_zones, zone, entry);
            zns_aor_dec_open(ns);
            zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_CLOSED);
        }
    }
}

static uint16_t zns_auto_open_zone(NvmeNamespace *ns, NvmeZone *zone)
{
    uint16_t status = NVME_SUCCESS;
    FemuCtrl *n = ns->ctrl;
    uint8_t zs = zns_get_zone_state(zone);
    uint32_t zone_idx = zns_zone_idx(ns, zone->d.zslba);

    if (zs == NVME_ZONE_STATE_EMPTY) {
        zns_auto_transition_zone(ns);
        status = zns_aor_check(ns, 1, 1);
        //Check whether the zone needs to be remapped
        zns_zone_blks_remaping(n,zone,zone_idx);
    } else if (zs == NVME_ZONE_STATE_CLOSED) {
        zns_auto_transition_zone(ns);
        status = zns_aor_check(ns, 0, 1);
    }
    return status;
}

static void zns_finalize_zoned_write(NvmeNamespace *ns, NvmeRequest *req, bool failed)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)&req->cmd;
    NvmeZone *zone;
    NvmeZonedResult *res = (NvmeZonedResult *)&req->cqe;
    uint64_t slba;
    uint32_t nlb;

    slba = le64_to_cpu(rw->slba);
    nlb = le16_to_cpu(rw->nlb) + 1;
    zone = zns_get_zone_by_slba(ns, slba);

    zone->d.wp += nlb;

    if (failed) {
        res->slba = 0;
    }

    if (zone->d.wp == zns_zone_wr_boundary(zone)) {
        switch (zns_get_zone_state(zone)) {
        case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        case NVME_ZONE_STATE_EXPLICITLY_OPEN:
            zns_aor_dec_open(ns);
            /* fall through */
        case NVME_ZONE_STATE_CLOSED:
            zns_aor_dec_active(ns);
            /* fall through */
        case NVME_ZONE_STATE_EMPTY:
            zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_FULL);
            /* fall through */
        case NVME_ZONE_STATE_FULL:
            break;
        default:
            assert(false);
        }
    }
}

// Add some function
// --------------------------------
static inline struct zns_ch *get_ch(struct zns_ssd *zns, struct ppa *ppa)
{
    return &(zns->ch[ppa->g.ch]);
}

static inline struct zns_fc *get_fc(struct zns_ssd *zns, struct ppa *ppa)
{
    struct zns_ch *ch = get_ch(zns, ppa);
    return &(ch->fc[ppa->g.fc]);
}

static inline struct zns_blk *get_blk(struct zns_ssd *zns, struct ppa *ppa)
{
    struct zns_fc *fc = get_fc(zns, ppa);
    return &(fc->blk[ppa->g.blk]);
}

static inline uint64_t zone_slba(FemuCtrl *n, uint32_t zone_idx)
{
    return (zone_idx) * n->zone_size;
}

static inline void check_addr(int a, int max)
{
   assert(a >= 0 && a < max);
}

static void advance_read_pointer(FemuCtrl *n)
{
    struct zns_ssd *zns = n->zns;
    struct write_pointer *wpp = &zns->wp;
    uint8_t num_ch = zns->num_ch;
    uint8_t num_lun = zns->num_lun;

    //printf("NUM CH: %"PRIu64"\n", wpp->ch);
    check_addr(wpp->ch, num_ch);
    wpp->ch++;

    if (wpp->ch != num_ch) {
        return;
    }

    /* Wrap-up, wpp->ch == num_ch */
    wpp->ch = 0;
    check_addr(wpp->lun, num_lun);
    wpp->lun++;
    if (wpp->lun == num_lun) {
        wpp->lun = 0;
        assert(wpp->ch == 0);
        assert(wpp->lun == 0);
    }
}

static inline struct ppa lpn_to_ppa(FemuCtrl *n, NvmeNamespace *ns, uint64_t lpn)
{

	uint32_t zone_idx = zns_zone_idx(ns, (lpn * 4096));

	struct zns_ssd *zns = n->zns;
	struct write_pointer *wpp = &zns->wp;
	//uint64_t num_ch = zns->num_ch;
	//uint64_t num_lun = zns->num_lun;
	struct ppa ppa = {0};

	//printf("OFFSET: %"PRIu64"\n\n", offset);
	//wpp->ch,lun
	ppa.g.ch = wpp->ch;
	ppa.g.fc = wpp->lun;
	// ppa.g.blk = zone_idx;
    ppa.g.blk = zns->ch[wpp->ch].fc[wpp->lun].blk[zone_idx].blk_id;


    return ppa;
}

static uint64_t zns_advance_status(FemuCtrl *n, struct nand_cmd *ncmd, struct ppa *ppa)
{
    int c = ncmd->cmd;

    struct zns_ssd *zns = n->zns;
    uint64_t nand_stime;
    uint64_t req_stime = (ncmd->stime == 0) ? \
        qemu_clock_get_ns(QEMU_CLOCK_REALTIME) : ncmd->stime;

    struct zns_fc *fc = get_fc(zns, ppa);

    uint64_t lat = 0;
    uint64_t read_delay = n->zns_params.zns_read;
    uint64_t write_delay = n->zns_params.zns_write;
    uint64_t erase_delay = 2000000;

    switch (c) {
    case NAND_READ:
        nand_stime = (fc->next_fc_avail_time < req_stime) ? req_stime : \
                     fc->next_fc_avail_time;
        fc->next_fc_avail_time = nand_stime + read_delay;
        lat = fc->next_fc_avail_time - req_stime;
	    break;

    case NAND_WRITE:
	    nand_stime = (fc->next_fc_avail_time < req_stime) ? req_stime : \
		            fc->next_fc_avail_time;
	    fc->next_fc_avail_time = nand_stime + write_delay;
	    lat = fc->next_fc_avail_time - req_stime;
	    break;

    case NAND_ERASE:
        nand_stime = (fc->next_fc_avail_time < req_stime) ? req_stime : \
                        fc->next_fc_avail_time;
        fc->next_fc_avail_time = nand_stime + erase_delay;
        lat = fc->next_fc_avail_time - req_stime;
        break;

    default:
        /* To silent warnings */
        ;
    }

    return lat;
}

static uint64_t zns_advance_zone_wp(NvmeNamespace *ns, NvmeZone *zone, uint32_t nlb)
{
    uint64_t result = zone->w_ptr;
    uint8_t zs;

    zone->w_ptr += nlb;

    if (zone->w_ptr < zns_zone_wr_boundary(zone)) {
        zs = zns_get_zone_state(zone);
        switch (zs) {
        case NVME_ZONE_STATE_EMPTY:
            zns_aor_inc_active(ns);
            /* fall through */
        case NVME_ZONE_STATE_CLOSED:
            zns_aor_inc_open(ns);
            zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_IMPLICITLY_OPEN);
        }
            }

    return result;
}

struct zns_zone_reset_ctx {
    NvmeRequest *req;
    NvmeZone    *zone;
};

static void zns_aio_zone_reset_cb(NvmeRequest *req, NvmeZone *zone)
{
    NvmeNamespace *ns = req->ns;
    FemuCtrl *n = ns->ctrl;
    int ch, lun;

    /* FIXME, We always assume reset SUCCESS */
    switch (zns_get_zone_state(zone)) {
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        /* fall through */
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        zns_aor_dec_open(ns);
        /* fall through */
    case NVME_ZONE_STATE_CLOSED:
        zns_aor_dec_active(ns);
        /* fall through */
    case NVME_ZONE_STATE_FULL:
        zone->w_ptr = zone->d.zslba;
        zone->d.wp = zone->w_ptr;
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_EMPTY);
    default:
        break;
    }
    struct zns_ssd *zns = n->zns;
    uint64_t num_ch = zns->num_ch;
    uint64_t num_lun = zns->num_lun;
    struct ppa ppa;

    uint32_t zone_blk_offset;
    uint32_t reset_zone_block_id = zns_zone_idx(ns, zone->d.zslba);
    femu_debug("reset_Zone_id:%u\n\r",reset_zone_block_id);
    for (ch = 0; ch < num_ch; ch++) {
        for (lun = 0; lun < num_lun; lun++) {
            ppa.g.ch = ch;
            ppa.g.fc = lun;
            ppa.g.blk = zns_zone_idx(ns, zone->d.zslba);
            // uint32_t reset_block_id = (ch*128)*8 + (lun*32)*8 + reset_zone_block_id;
            zone_blk_offset = ch  + lun * num_ch ;
            zone->blks[zone_blk_offset]->erase_count += 1;
            zone->erase_count += 1;
            zns->ch[ch].fc[lun].erase_count += 1;
            femu_debug("reset_block_id:%lu,erase_count:%lu\n\r",zone->blks[zone_blk_offset]->blk_id,zone->blks[zone_blk_offset]->erase_count);
            struct nand_cmd erase;
            erase.cmd = NAND_ERASE; 
            erase.stime = 0;
            zns_advance_status(n, &erase, &ppa);
        }
    }
    femu_debug("Zone%u:reset_count:%lu\n\r",reset_zone_block_id,zone->erase_count);
}

typedef uint16_t (*op_handler_t)(NvmeNamespace *, NvmeZone *, NvmeZoneState,
                                 NvmeRequest *);

enum NvmeZoneProcessingMask {
    NVME_PROC_CURRENT_ZONE    = 0,
    NVME_PROC_OPENED_ZONES    = 1 << 0,
    NVME_PROC_CLOSED_ZONES    = 1 << 1,
    NVME_PROC_READ_ONLY_ZONES = 1 << 2,
    NVME_PROC_FULL_ZONES      = 1 << 3,
};

static uint16_t zns_open_zone(NvmeNamespace *ns, NvmeZone *zone,
                              NvmeZoneState state, NvmeRequest *req)
{
    uint16_t status;

    switch (state) {
    case NVME_ZONE_STATE_EMPTY:
        status = zns_aor_check(ns, 1, 0);
        if (status != NVME_SUCCESS) {
            return status;
        }
        zns_aor_inc_active(ns);
        /* fall through */
    case NVME_ZONE_STATE_CLOSED:
        status = zns_aor_check(ns, 0, 1);
        if (status != NVME_SUCCESS) {
            if (state == NVME_ZONE_STATE_EMPTY) {
                zns_aor_dec_active(ns);
            }
            return status;
        } 
        zns_aor_inc_open(ns);
        /* fall through */
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_EXPLICITLY_OPEN);
        /* fall through */
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        return NVME_SUCCESS;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }
}

static uint16_t zns_close_zone(NvmeNamespace *ns, NvmeZone *zone,
                               NvmeZoneState state, NvmeRequest *req)
{
    switch (state) {
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        /* fall through */
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        zns_aor_dec_open(ns);
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_CLOSED);
        /* fall through */
    case NVME_ZONE_STATE_CLOSED:
        return NVME_SUCCESS;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }
}

static uint16_t zns_finish_zone(NvmeNamespace *ns, NvmeZone *zone,
                                NvmeZoneState state, NvmeRequest *req)
{
    switch (state) {
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        /* fall through */
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
        zns_aor_dec_open(ns);
        /* fall through */
    case NVME_ZONE_STATE_CLOSED:
        zns_aor_dec_active(ns);
        /* fall through */
    case NVME_ZONE_STATE_EMPTY:
        zone->w_ptr = zns_zone_wr_boundary(zone);
        zone->d.wp = zone->w_ptr;
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_FULL);
        /* fall through */
    case NVME_ZONE_STATE_FULL:
        return NVME_SUCCESS;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }
}

static uint16_t zns_reset_zone(NvmeNamespace *ns, NvmeZone *zone,
                               NvmeZoneState state, NvmeRequest *req)
{
    switch (state) {
    case NVME_ZONE_STATE_EMPTY:
        return NVME_SUCCESS;
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
    case NVME_ZONE_STATE_CLOSED:
    case NVME_ZONE_STATE_FULL:
        break;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }

    zns_aio_zone_reset_cb(req, zone);

    return NVME_SUCCESS;
}

static uint16_t zns_offline_zone(NvmeNamespace *ns, NvmeZone *zone,
                                 NvmeZoneState state, NvmeRequest *req)
{
    switch (state) {
    case NVME_ZONE_STATE_READ_ONLY:
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_OFFLINE);
        /* fall through */
    case NVME_ZONE_STATE_OFFLINE:
        return NVME_SUCCESS;
    default:
        return NVME_ZONE_INVAL_TRANSITION;
    }
}

static uint16_t zns_set_zd_ext(NvmeNamespace *ns, NvmeZone *zone)
{
    uint16_t status;
    uint8_t state = zns_get_zone_state(zone);

    if (state == NVME_ZONE_STATE_EMPTY) {
        status = zns_aor_check(ns, 1, 0);
        if (status != NVME_SUCCESS) {
            return status;
        }
        zns_aor_inc_active(ns);
        zone->d.za |= NVME_ZA_ZD_EXT_VALID;
        zns_assign_zone_state(ns, zone, NVME_ZONE_STATE_CLOSED);
        return NVME_SUCCESS;
    }

    return NVME_ZONE_INVAL_TRANSITION;
}

static uint16_t zns_bulk_proc_zone(NvmeNamespace *ns, NvmeZone *zone,
                                   enum NvmeZoneProcessingMask proc_mask,
                                   op_handler_t op_hndlr, NvmeRequest *req)
{
    uint16_t status = NVME_SUCCESS;
    NvmeZoneState zs = zns_get_zone_state(zone);
    bool proc_zone;

    switch (zs) {
    case NVME_ZONE_STATE_IMPLICITLY_OPEN:
    case NVME_ZONE_STATE_EXPLICITLY_OPEN:
        proc_zone = proc_mask & NVME_PROC_OPENED_ZONES;
        break;
    case NVME_ZONE_STATE_CLOSED:
        proc_zone = proc_mask & NVME_PROC_CLOSED_ZONES;
        break;
    case NVME_ZONE_STATE_READ_ONLY:
        proc_zone = proc_mask & NVME_PROC_READ_ONLY_ZONES;
        break;
    case NVME_ZONE_STATE_FULL:
        proc_zone = proc_mask & NVME_PROC_FULL_ZONES;
        break;
    default:
        proc_zone = false;
    }

    if (proc_zone) {
        status = op_hndlr(ns, zone, zs, req);
    }

    return status;
}

static uint16_t zns_do_zone_op(NvmeNamespace *ns, NvmeZone *zone,
                               enum NvmeZoneProcessingMask proc_mask,
                               op_handler_t op_hndlr, NvmeRequest *req)
{
    FemuCtrl *n = ns->ctrl;
    NvmeZone *next;
    uint16_t status = NVME_SUCCESS;
    int i;

    if (!proc_mask) {
        status = op_hndlr(ns, zone, zns_get_zone_state(zone), req);
    } else {
        if (proc_mask & NVME_PROC_CLOSED_ZONES) {
            QTAILQ_FOREACH_SAFE(zone, &n->closed_zones, entry, next) {
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr, req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }
        }
        if (proc_mask & NVME_PROC_OPENED_ZONES) {
            QTAILQ_FOREACH_SAFE(zone, &n->imp_open_zones, entry, next) {
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr,
                                             req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }

            QTAILQ_FOREACH_SAFE(zone, &n->exp_open_zones, entry, next) {
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr,
                                             req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }
        }
        if (proc_mask & NVME_PROC_FULL_ZONES) {
            QTAILQ_FOREACH_SAFE(zone, &n->full_zones, entry, next) {
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr, req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }
        }

        if (proc_mask & NVME_PROC_READ_ONLY_ZONES) {
            for (i = 0; i < n->num_zones; i++, zone++) {
                status = zns_bulk_proc_zone(ns, zone, proc_mask, op_hndlr,
                                             req);
                if (status && status != NVME_NO_COMPLETE) {
                    goto out;
                }
            }
        }
    }

out:
    return status;
}

static uint16_t zns_get_mgmt_zone_slba_idx(FemuCtrl *n, NvmeCmd *c,
                                           uint64_t *slba, uint32_t *zone_idx)
{
    NvmeNamespace *ns = &n->namespaces[0];
    uint32_t dw10 = le32_to_cpu(c->cdw10);
    uint32_t dw11 = le32_to_cpu(c->cdw11);

    if (!n->zoned) {
        return NVME_INVALID_OPCODE | NVME_DNR;
    }

    *slba = ((uint64_t)dw11) << 32 | dw10;
    if (unlikely(*slba >= ns->id_ns.nsze)) {
        *slba = 0;
        return NVME_LBA_RANGE | NVME_DNR;
    }

    *zone_idx = zns_zone_idx(ns, *slba);
    assert(*zone_idx < n->num_zones);

    return NVME_SUCCESS;
}

static uint16_t zns_zone_mgmt_send(FemuCtrl *n, NvmeRequest *req)
{
    NvmeCmd *cmd = (NvmeCmd *)&req->cmd;
    NvmeNamespace *ns = req->ns;
    uint64_t prp1 = le64_to_cpu(cmd->dptr.prp1);
    uint64_t prp2 = le64_to_cpu(cmd->dptr.prp2);
    NvmeZone *zone;
    uintptr_t *resets;
    uint8_t *zd_ext;
    uint32_t dw13 = le32_to_cpu(cmd->cdw13);
    uint64_t slba = 0;
    uint32_t zone_idx = 0;
    uint16_t status;
    uint8_t action;
    bool all;
    enum NvmeZoneProcessingMask proc_mask = NVME_PROC_CURRENT_ZONE;

    action = dw13 & 0xff;
    all = dw13 & 0x100;

    req->status = NVME_SUCCESS;

    if (!all) {
        status = zns_get_mgmt_zone_slba_idx(n, cmd, &slba, &zone_idx);
        if (status) {
            return status;
        }
    }

    zone = &n->zone_array[zone_idx];
    if (slba != zone->d.zslba) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    switch (action) {
    case NVME_ZONE_ACTION_OPEN:
        if (all) {
            proc_mask = NVME_PROC_CLOSED_ZONES;
        }
        status = zns_do_zone_op(ns, zone, proc_mask, zns_open_zone, req);
        break;
    case NVME_ZONE_ACTION_CLOSE:
        if (all) {
            proc_mask = NVME_PROC_OPENED_ZONES;
        }
        status = zns_do_zone_op(ns, zone, proc_mask, zns_close_zone, req);
        break;
    case NVME_ZONE_ACTION_FINISH:
        if (all) {
            proc_mask = NVME_PROC_OPENED_ZONES | NVME_PROC_CLOSED_ZONES;
        }
        status = zns_do_zone_op(ns, zone, proc_mask, zns_finish_zone, req);
        break;
    case NVME_ZONE_ACTION_RESET:
        resets = (uintptr_t *)&req->opaque;

        if (all) {
            proc_mask = NVME_PROC_OPENED_ZONES | NVME_PROC_CLOSED_ZONES |
                NVME_PROC_FULL_ZONES;
        }
        *resets = 1;
        status = zns_do_zone_op(ns, zone, proc_mask, zns_reset_zone, req);
        (*resets)--;
        return NVME_SUCCESS;
    case NVME_ZONE_ACTION_OFFLINE:
        if (all) {
            proc_mask = NVME_PROC_READ_ONLY_ZONES;
        }
        status = zns_do_zone_op(ns, zone, proc_mask, zns_offline_zone, req);
        break;
    case NVME_ZONE_ACTION_SET_ZD_EXT:
        if (all || !n->zd_extension_size) {
            return NVME_INVALID_FIELD | NVME_DNR;
        }
        zd_ext = zns_get_zd_extension(ns, zone_idx);
        status = dma_write_prp(n, (uint8_t *)zd_ext, n->zd_extension_size, prp1,
                               prp2);
        if (status) {
            return status;
        }
        status = zns_set_zd_ext(ns, zone);
        if (status == NVME_SUCCESS) {
            return status;
        }
        break;
    default:
        status = NVME_INVALID_FIELD;
    }

    if (status) {
        status |= NVME_DNR;
    }

    return status;
}

static bool zns_zone_matches_filter(uint32_t zafs, NvmeZone *zl)
{
    NvmeZoneState zs = zns_get_zone_state(zl);

    switch (zafs) {
    case NVME_ZONE_REPORT_ALL:
        return true;
    case NVME_ZONE_REPORT_EMPTY:
        return zs == NVME_ZONE_STATE_EMPTY;
    case NVME_ZONE_REPORT_IMPLICITLY_OPEN:
        return zs == NVME_ZONE_STATE_IMPLICITLY_OPEN;
    case NVME_ZONE_REPORT_EXPLICITLY_OPEN:
        return zs == NVME_ZONE_STATE_EXPLICITLY_OPEN;
    case NVME_ZONE_REPORT_CLOSED:
        return zs == NVME_ZONE_STATE_CLOSED;
    case NVME_ZONE_REPORT_FULL:
        return zs == NVME_ZONE_STATE_FULL;
    case NVME_ZONE_REPORT_READ_ONLY:
        return zs == NVME_ZONE_STATE_READ_ONLY;
    case NVME_ZONE_REPORT_OFFLINE:
        return zs == NVME_ZONE_STATE_OFFLINE;
    default:
        return false;
    }
}

static uint16_t zns_zone_mgmt_recv(FemuCtrl *n, NvmeRequest *req)
{
    NvmeCmd *cmd = (NvmeCmd *)&req->cmd;
    NvmeNamespace *ns = req->ns;
    uint64_t prp1 = le64_to_cpu(cmd->dptr.prp1);
    uint64_t prp2 = le64_to_cpu(cmd->dptr.prp2);
    /* cdw12 is zero-based number of dwords to return. Convert to bytes */
    uint32_t data_size = (le32_to_cpu(cmd->cdw12) + 1) << 2;
    uint32_t dw13 = le32_to_cpu(cmd->cdw13);
    uint32_t zone_idx, zra, zrasf, partial;
    uint64_t max_zones, nr_zones = 0;
    uint16_t status;
    uint64_t slba, capacity = zns_ns_nlbas(ns);
    NvmeZoneDescr *z;
    NvmeZone *zone;
    NvmeZoneReportHeader *header;
    void *buf, *buf_p;
    size_t zone_entry_sz;

    req->status = NVME_SUCCESS;

    status = zns_get_mgmt_zone_slba_idx(n, cmd, &slba, &zone_idx);
    if (status) {
        return status;
    }

    zra = dw13 & 0xff;
    if (zra != NVME_ZONE_REPORT && zra != NVME_ZONE_REPORT_EXTENDED) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    if (zra == NVME_ZONE_REPORT_EXTENDED && !n->zd_extension_size) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    zrasf = (dw13 >> 8) & 0xff;
    if (zrasf > NVME_ZONE_REPORT_OFFLINE) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    if (data_size < sizeof(NvmeZoneReportHeader)) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    status = nvme_check_mdts(n, data_size);
    if (status) {
        return status;
    }

    partial = (dw13 >> 16) & 0x01;

    zone_entry_sz = sizeof(NvmeZoneDescr);
    if (zra == NVME_ZONE_REPORT_EXTENDED) {
        zone_entry_sz += n->zd_extension_size;
    }

    max_zones = (data_size - sizeof(NvmeZoneReportHeader)) / zone_entry_sz;
    buf = g_malloc0(data_size);

    zone = &n->zone_array[zone_idx];
    for (; slba < capacity; slba += n->zone_size) {
        if (partial && nr_zones >= max_zones) {
            break;
        }
        if (zns_zone_matches_filter(zrasf, zone++)) {
            nr_zones++;
        }
    }
    header = (NvmeZoneReportHeader *)buf;
    header->nr_zones = cpu_to_le64(nr_zones);

    buf_p = buf + sizeof(NvmeZoneReportHeader);
    for (; zone_idx < n->num_zones && max_zones > 0; zone_idx++) {
        zone = &n->zone_array[zone_idx];
        if (zns_zone_matches_filter(zrasf, zone)) {
            z = (NvmeZoneDescr *)buf_p;
            buf_p += sizeof(NvmeZoneDescr);

            z->zt = zone->d.zt;
            z->zs = zone->d.zs;
            z->zcap = cpu_to_le64(zone->d.zcap);
            z->zslba = cpu_to_le64(zone->d.zslba);
            z->za = zone->d.za;

            if (zns_wp_is_valid(zone)) {
                z->wp = cpu_to_le64(zone->d.wp);
            } else {
                z->wp = cpu_to_le64(~0ULL);
            }

            if (zra == NVME_ZONE_REPORT_EXTENDED) {
                if (zone->d.za & NVME_ZA_ZD_EXT_VALID) {
                    memcpy(buf_p, zns_get_zd_extension(ns, zone_idx),
                           n->zd_extension_size);
                }
                buf_p += n->zd_extension_size;
            }

            max_zones--;
        }
    }

    status = dma_read_prp(n, (uint8_t *)buf, data_size, prp1, prp2);

    g_free(buf);

    return status;
}

static inline bool nvme_csi_has_nvm_support(NvmeNamespace *ns)
{
    switch (ns->ctrl->csi) {
    case NVME_CSI_NVM:
    case NVME_CSI_ZONED:
        return true;
    }

    return false;
}

static inline uint16_t zns_check_bounds(NvmeNamespace *ns, uint64_t slba,
                                        uint32_t nlb)
{
    uint64_t nsze = le64_to_cpu(ns->id_ns.nsze);

    if (unlikely(UINT64_MAX - slba < nlb || slba + nlb > nsze)) {
        return NVME_LBA_RANGE | NVME_DNR;
    }

    return NVME_SUCCESS;
}

static uint16_t zns_map_dptr(FemuCtrl *n, size_t len, NvmeRequest *req)
{
    uint64_t prp1, prp2;//存储传输指针的物理地址

    switch (req->cmd.psdt) {//PSDT，Parameter Service Data Transfer）
    case NVME_PSDT_PRP:// PRP（Physical Region Page）
        prp1 = le64_to_cpu(req->cmd.dptr.prp1);
        prp2 = le64_to_cpu(req->cmd.dptr.prp2);

        //调用 nvme_map_prp 函数，传递相关参数（&req->qsg, &req->iov, prp1, prp2, len, n）进行PRP映射，将PRP映射为主机的内存区域，同时更新请求的散列段和I/O向量
        return nvme_map_prp(&req->qsg, &req->iov, prp1, prp2, len, n);
    default:
        return NVME_INVALID_FIELD;
    }
}

static uint16_t zns_do_write(FemuCtrl *n, NvmeRequest *req, bool append,
                             bool wrz)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)&req->cmd;
    NvmeNamespace *ns = req->ns;
    uint64_t slba = le64_to_cpu(rw->slba);
    uint32_t nlb = (uint32_t)le16_to_cpu(rw->nlb) + 1;
    uint64_t data_size = zns_l2b(ns, nlb);
    uint64_t data_offset;
    NvmeZone *zone;
    NvmeZonedResult *res = (NvmeZonedResult *)&req->cqe;
    uint16_t status;

    assert(n->zoned);
    req->is_write = true;

    if (!wrz) {
        status = nvme_check_mdts(n, data_size);
        if (status) {
            goto err;
        }
    }

    status = zns_check_bounds(ns, slba, nlb);
    if (status) {
        goto err;
    }

    zone = zns_get_zone_by_slba(ns, slba);

    status = zns_check_zone_write(n, ns, zone, slba, nlb, append);
    if (status) {
        goto err;
    }

    status = zns_auto_open_zone(ns, zone);
    if (status) {
        goto err;
    }

    if (append) {
        slba = zone->w_ptr;
    }

    res->slba = zns_advance_zone_wp(ns, zone, nlb);

    data_offset = zns_l2b(ns, slba);

    if (!wrz) {
        status = zns_map_dptr(n, data_size, req);
        if (status) {
            goto err;
        }

        backend_rw(n->mbe, &req->qsg, &data_offset, req->is_write);
    }
    // femu_debug("zns_do_write called\n\r");
    zns_finalize_zoned_write(ns, req, false);
    return NVME_SUCCESS;

err:
    printf("****************Append Failed***************\n");
    return status | NVME_DNR;
}

static uint16_t zns_admin_cmd(FemuCtrl *n, NvmeCmd *cmd)
{
    switch (cmd->opcode) {
    default:
        return NVME_INVALID_OPCODE | NVME_DNR;
    }
}

static inline uint16_t zns_zone_append(FemuCtrl *n, NvmeRequest *req)
{
    return zns_do_write(n, req, true, false);
}

static uint16_t zns_check_dulbe(NvmeNamespace *ns, uint64_t slba, uint32_t nlb)
{
    return NVME_SUCCESS;
}

static uint16_t zns_read(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                         NvmeRequest *req)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)&req->cmd;
    uint64_t slba = le64_to_cpu(rw->slba);
    uint32_t nlb = (uint32_t)le16_to_cpu(rw->nlb) + 1;
    uint64_t data_size = zns_l2b(ns, nlb);
    uint64_t data_offset;
    uint16_t status;

    assert(n->zoned);
    req->is_write = false;

    status = nvme_check_mdts(n, data_size);
    if (status) {
        goto err;
    }

    status = zns_check_bounds(ns, slba, nlb);
    if (status) {
        goto err;
    }

    status = zns_check_zone_read(ns, slba, nlb);
    if (status) {
        goto err;
    }

    status = zns_map_dptr(n, data_size, req);
    if (status) {
        goto err;
    }

    if (NVME_ERR_REC_DULBE(n->features.err_rec)) {
        status = zns_check_dulbe(ns, slba, nlb);
        if (status) {
            goto err;
        }
    }

    data_offset = zns_l2b(ns, slba);

    backend_rw(n->mbe, &req->qsg, &data_offset, req->is_write);

    uint64_t slpn = (slba) / 4096;
    uint64_t elpn = (slba + nlb - 1) / 4096;
    uint64_t lpn;
    struct ppa ppa;
    uint64_t sublat,maxlat=0;

    for (lpn = slpn; lpn <= elpn; lpn++) {
        ppa = lpn_to_ppa(n, ns, lpn);
        advance_read_pointer(n);

        struct nand_cmd read;
        read.cmd = NAND_READ;
        read.stime = req->stime;

        sublat = zns_advance_status(n, &read, &ppa);
        maxlat = (sublat > maxlat) ? sublat : maxlat;
    }

    req->reqlat = maxlat;
    req->expire_time += maxlat;

    return NVME_SUCCESS;

err:
    return status | NVME_DNR;
}

static uint16_t zns_write(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                          NvmeRequest *req)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)cmd;//NvmeRwCmd 是用于表示 NVMe 读写命令的结构体
    uint64_t slba = le64_to_cpu(rw->slba);//start logical block address
    uint32_t nlb = (uint32_t)le16_to_cpu(rw->nlb) + 1;//sector
    uint64_t data_size = zns_l2b(ns, nlb);
    uint64_t data_offset;
    NvmeZone *zone;
    NvmeZonedResult *res = (NvmeZonedResult *)&req->cqe;
    uint16_t status;

    assert(n->zoned);
    req->is_write = true;

    status = nvme_check_mdts(n, data_size);//Memory Data Transfer Size
    if (status) {
        goto err;
    }

    status = zns_check_bounds(ns, slba, nlb);//檢查slba是否在有效範圍内
    if (status) {
        goto err;
    }
    

    zone = zns_get_zone_by_slba(ns, slba);//获得slba相应的zone区
    // femu_debug("slba=%lu, wp=%lu, nlb=%u\n\r", slba, zone->w_ptr, nlb);

    status = zns_check_zone_write(n, ns, zone, slba, nlb, false);//检查是否可以在指定的区域（zone）中进行写操作
    if (status) {
        goto err;
    }

    status = zns_auto_open_zone(ns, zone);//打开指定zone
    if (status) {
        goto err;
    }
    //将请求的写指针赋值给res-slba，然后更新zone的写指针zone->w_ptr += nlb;
    res->slba = zns_advance_zone_wp(ns, zone, nlb);

    //convert an LBA to the equivalent in bytes
    data_offset = zns_l2b(ns, slba);
    status = zns_map_dptr(n, data_size, req);//用于将NVMe命令的数据传输指针（Data Pointer，DPTR）映射到主机的内存区域,将prp内存地址保存在req->qsg
    if (status) {
        goto err;
    }

    //在 FEMU 后端执行读写操作，将数据在主机内存和 FEMU 逻辑空间之间传输，根据 FEMU 模式更新逻辑块偏移
    backend_rw(n->mbe, &req->qsg, &data_offset, req->is_write);
    //在 ZNS 写操作完成后，更新区域的写指针，处理区域状态的转换，以及根据写操作是否失败来设置结果中的 SLBA
    zns_finalize_zoned_write(ns, req, false);
    uint64_t slpn = (slba) / 4096;//逻辑块地址为什么除以4096 -->slba*512/(8*512)才是第一个lpn吧
    uint64_t elpn = (slba + nlb - 1) / 4096;//(slba + nlb - 1) *512/ 4096

    uint64_t lpn;
    struct ppa ppa;
    uint64_t sublat,maxlat=0;


    for (lpn = slpn; lpn <= elpn; lpn++) {
        ppa = lpn_to_ppa(n, ns, lpn);
        advance_read_pointer(n);
        struct nand_cmd write;
        write.cmd = NAND_WRITE;
        write.stime = req->stime;

        sublat = zns_advance_status(n, &write, &ppa);
        maxlat = (sublat > maxlat) ? sublat : maxlat;
    }

    req->reqlat = maxlat;
    req->expire_time += maxlat;
    return NVME_SUCCESS;

err:
    femu_err("*********ZONE WRITE FAILED*********\n");
    return status | NVME_DNR;
}

static uint16_t zns_io_cmd(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                           NvmeRequest *req)
{
    switch (cmd->opcode) {
    case NVME_CMD_READ:
        return zns_read(n, ns, cmd, req);
    case NVME_CMD_WRITE:
        return zns_write(n, ns, cmd, req);
    case NVME_CMD_ZONE_MGMT_SEND:
        return zns_zone_mgmt_send(n, req);
    case NVME_CMD_ZONE_MGMT_RECV:
        return zns_zone_mgmt_recv(n, req);
    case NVME_CMD_ZONE_APPEND:
        return zns_zone_append(n, req);
    }

    return NVME_INVALID_OPCODE | NVME_DNR;
}

static void zns_set_ctrl_str(FemuCtrl *n)
{
    static int fsid_zns = 0;
    const char *zns_mn = "FEMU ZNS-SSD Controller";
    const char *zns_sn = "vZNSSD";

    nvme_set_ctrl_name(n, zns_mn, zns_sn, &fsid_zns);                        
}

static void zns_set_ctrl(FemuCtrl *n)
{
    uint8_t *pci_conf = n->parent_obj.config;

    zns_set_ctrl_str(n);
    pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_INTEL);
    pci_config_set_device_id(pci_conf, 0x5845);
}

static void zns_init_blk(struct zns_blk *blk, uint8_t ch_id, uint8_t lun_id, uint8_t blk_offset)
{
    blk->next_blk_avail_time = 0;
    blk->erase_count = 0;
    /*
    blk_offset refers to blocks with the same offset in each chip
    blk_id = ch_id * blk_per_ch + lun_id * blk_per_lun + blk_offset
    */
    blk->blk_id = ch_id * BLKS_PER_CH + lun_id * BLKS_PER_LUN + blk_offset; 
}

static void zns_init_fc(struct zns_fc *fc, uint8_t ch_id, uint8_t lun_id)
{
    fc->blk = g_malloc0(sizeof(struct zns_blk) * BLKS_PER_LUN);
    for (int i = 0; i < BLKS_PER_LUN; i++) {
        zns_init_blk(&fc->blk[i],ch_id,lun_id,i);
    }
    fc->next_fc_avail_time = 0;
    fc->erase_count = 0;
}

static void zns_init_ch(struct zns_ch *ch, uint8_t num_lun, uint8_t ch_id)
{
    ch->fc = g_malloc0(sizeof(struct zns_fc) * num_lun);
    for (int i = 0; i < num_lun; i++) {
        zns_init_fc(&ch->fc[i], ch_id, i);
    }
    ch->next_ch_avail_time = 0;
}
//add it to init zone blks mapping
static void zns_init_zone_blks_map(FemuCtrl *n)
{
    NvmeNamespace *ns = &n->namespaces[0];
    struct zns_ssd *ssd = n->zns;
    struct write_pointer *wpp = &ssd->wp;
    NvmeZone *zones = n->zone_array;

    for (int i = 0; i < n->num_zones; i++)
    {
        uint32_t zone_idx = zns_zone_idx(ns, zones[i].d.zslba);
        //循环zone的blk
        for (int j = 0; j < BLKS_PER_ZONE; j++)
        {
            // struct zns_blk **zone_blk = zones[i].blks + sizeof(struct zns_blk)
            zones[i].blks[j] = &(ssd->ch[wpp->ch].fc[wpp->lun].blk[zone_idx]);
            //初始化blk存储池
            ssd->zns_blk_pool.blks_pool[zones[i].blks[j]->blk_id] = 1;
            advance_read_pointer(n);
        }
    }
    femu_debug("zone_blk映射初始化成功\n\r");
}

//Check whether the zone needs to be remapped. If the zone wear is too large( >= ERASE_THRESHOLD), 
//complete the remapping
static void zns_zone_blks_remaping(FemuCtrl *n, NvmeZone *zone ,uint32_t zone_id)
{
    NvmeZone *zones = n->zone_array;
    struct zns_ssd *ssd = n->zns;
    // zone = &n->zone_array[zone_idx];

    // return (n->zone_size_log2 > 0 ? slba >> n->zone_size_log2 : slba / n->zone_size);

    struct zns_blk *min_erase_blk = NULL;//最小擦除次数 blk的物理blk id

    if(zone->erase_count >= ERASE_THRESHOLD || zone->blks[0] == NULL || (zone->erase_count - ssd->zns_blk_pool.zone_min_erase_count) > REMAP_BETWEEN_ZONES)
    {
        for (int i = 0; i < n->num_zones; i++, zones++) 
        {
            uint8_t zs = zns_get_zone_state(zones);
            if (zs  != NVME_ZONE_STATE_EMPTY)
            {
                continue;
            }
            //首先循环zone映射的blk，释放其blk（让其blk_pool的值设置为0）,然后将映射取消 
            for (int j = 0; j < BLKS_PER_ZONE; j++)
            {
                if (zones->blks[j] != NULL)
                {
                    ssd->zns_blk_pool.blks_pool[zones->blks[j]->blk_id] = 0;
                    zones->blks[j] = NULL;
                }
            }
            zones->erase_count = 0;
        }
        //remaping 
        //步骤：1.从每个lun中找出擦除次数最小的blk，然后更新存储池，让zone映射指向包含的blk
        zone->erase_count = 0;
        for (int ch= 0; ch < ssd->num_ch; ch++)
        {
            for (int lun = 0; lun < ssd->num_lun; lun++)
            {
                // min_erase_blk = ssd->ch[k].fc[m].blk[0];
                for (int n = 0; n < BLKS_PER_LUN; n++)
                {
                    if (ssd->zns_blk_pool.blks_pool[ssd->ch[ch].fc[lun].blk[n].blk_id] == 1)
                    {
                        continue;
                        /* code */
                    }else
                    {
                        if (min_erase_blk != NULL && ssd->ch[ch].fc[lun].blk[n].erase_count < min_erase_blk->erase_count)
                        {
                            min_erase_blk = &(ssd->ch[ch].fc[lun].blk[n]);
                            // femu_debug("select ch:%u lun:%u blk_id:%lu ec:%lu -> zone:%u\n\r",ch,lun,min_erase_blk->blk_id,min_erase_blk->erase_count,zone_id);
                        }else if (min_erase_blk == NULL)
                        {
                            min_erase_blk = &(ssd->ch[ch].fc[lun].blk[n]);
                        }
                    }
                }
                // femu_debug("select ch:%u lun:%u blk_id:%lu ec:%lu -> zone:%u\n\r",ch,lun,min_erase_blk->blk_id,min_erase_blk->erase_count,zone_id);
                zone->blks[lun + ch*ssd->num_lun] = min_erase_blk;
                ssd->zns_blk_pool.blks_pool[min_erase_blk->blk_id] = 1;
                zone->erase_count += min_erase_blk->erase_count;
                min_erase_blk = NULL;
                // femu_debug("zone:%u erase_count:%lu\n\r",zone_id,zone->erase_count);
            }
        }
        //更新zone 中最小的zone的擦除次数    
        if (zone->erase_count < ssd->zns_blk_pool.zone_min_erase_count)
        {
            ssd->zns_blk_pool.zone_min_erase_count = zone->erase_count;
        }
        femu_debug("zone:%u remap successfully,erase_count:%lu\n\r",zone_id,zone->erase_count);  
        return;
    }else
    {
        femu_debug("zone:%u remap failed: can not satisfied remap case\n\r",zone_id);  
        return;
    }
}


static void zns_init_params(FemuCtrl *n)
{
    struct zns_ssd *id_zns;
    int i;

    id_zns = g_malloc0(sizeof(struct zns_ssd));
    id_zns->num_ch = n->zns_params.zns_num_ch;
    id_zns->num_lun = n->zns_params.zns_num_lun;
    id_zns->ch = g_malloc0(sizeof(struct zns_ch) * id_zns->num_ch);
    //addition
    id_zns->zns_blk_pool = *(struct zns_blk_pool *)g_malloc0(sizeof(struct zns_blk_pool));
    id_zns->zns_blk_pool.zone_min_erase_count = 0;
    id_zns->zns_blk_pool.blks_pool = g_malloc0(sizeof(uint8_t) * BLKS_PER_CH * id_zns->num_ch);
    memset(id_zns->zns_blk_pool.blks_pool, 0, sizeof(uint8_t) * BLKS_PER_CH * id_zns->num_ch);
    
    for (i =0; i < id_zns->num_ch; i++) {
        zns_init_ch(&id_zns->ch[i], id_zns->num_lun,i);
    }
    id_zns->wp.ch = 0;
    id_zns->wp.lun = 0;
    n->zns = id_zns;
}

static int zns_init_zone_cap(FemuCtrl *n)
{
    n->zoned = true;
    n->zasl_bs = NVME_DEFAULT_MAX_AZ_SIZE;//（Zone Active Size Limit)
    n->zone_size_bs = NVME_DEFAULT_ZONE_SIZE;
    n->zone_cap_bs = 0;
    n->cross_zone_read = false;
    n->max_active_zones = 0;
    n->max_open_zones = 0;
    n->zd_extension_size = 0;

    return 0;
}

static int zns_start_ctrl(FemuCtrl *n)
{
    /* Coperd: let's fail early before anything crazy happens */
    assert(n->page_size == 4096);

    if (!n->zasl_bs) {
        n->zasl = n->mdts;
    } else {
        if (n->zasl_bs < n->page_size) {
            femu_err("ZASL too small (%dB), must >= 1 page (4K)\n", n->zasl_bs);
            return -1;
        }
        n->zasl = 31 - clz32(n->zasl_bs / n->page_size);
    }

    return 0;
}

static void zns_init(FemuCtrl *n, Error **errp)
{
    NvmeNamespace *ns = &n->namespaces[0];
    // NvmeZone *zones = n->zone_array;
    
    zns_set_ctrl(n);

    zns_init_zone_cap(n);

    if (zns_init_zone_geometry(ns, errp) != 0) {
        return;
    }
    zns_init_zone_identify(n, ns, 0);
    zns_init_params(n);
    //add to complete blks mapping
    zns_init_zone_blks_map(n);
    femu_debug("zns_init_zone_blks_map success\n\r");
}

static void zns_exit(FemuCtrl *n)
{
    /*
     * Release any extra resource (zones) allocated for ZNS mode
     */
}

int nvme_register_znssd(FemuCtrl *n)
{
    n->ext_ops = (FemuExtCtrlOps) {
        .state            = NULL,
        .init             = zns_init,
        .exit             = zns_exit,
        .rw_check_req     = NULL,
        .start_ctrl       = zns_start_ctrl,
        .admin_cmd        = zns_admin_cmd,
        .io_cmd           = zns_io_cmd,
        .get_log          = NULL,
    };

    return 0;
}
