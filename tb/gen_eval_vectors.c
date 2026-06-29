/* gen_eval_vectors.c ─────────────────────────────────────────────────────────
 * Tick-domain golden model for sched_candidate_eval_lane.
 * 1 tick = 11264 clock cycles.
 *
 * Outputs 88 space-separated integers per line to stdout.
 * Redirect: ./gen_eval_vectors > eval_vectors.txt
 *
 * Field layout (0-indexed):
 *  [0]        plan_type       (0=PAIR, 1=SPLIT, 2=SOLO)
 *  [1]        cluster_a
 *  [2]        enable_s2pf
 *  [3]        single_latest_s2pf
 *  [4]        force_shape_a
 *  [5]        force_shape_b
 *  [6]        forced_s1a
 *  [7]        forced_s3a
 *  [8]        forced_s1b
 *  [9]        forced_s3b
 *  [10]       cost_only_tie
 *  [11]       side_a_valid
 *  [12]       side_b_valid
 *  [13]       start_a (ticks)
 *  [14]       start_b (ticks)
 *  [15]       eid_a
 *  [16]       eid_b
 *  [17]       ntok_a
 *  [18]       ntok_b
 *  [19]       tok_start_a
 *  [20]       tok_start_b
 *  [21]       sw_a
 *  [22]       dn_a
 *  [23]       sw_b
 *  [24]       dn_b
 *  [25]       shape_t0 (ticks)
 *  [26]       rem_len
 *  [27]       rem0_eid
 *  [28]       rem0_ntok
 *  [29]       rem1_ntok
 *  [30]       total_conc (ticks)
 *  [31]       max_conc (ticks)
 *  [32..50]   base_snap_a: valid, task_start, task_end, dma1_end, s1_end,
 *             s2_end, dma3_end, s3_end, s4_start, bw_s1, bw_s3,
 *             s2pf_valid, s2pf_start, s2pf_end, s2pf_bw, ntok,
 *             pf_eid, pf_end, pf_full
 *  [51..69]   base_snap_b: same 19 fields
 *  [70]       exp_bw_ok
 *  [71]       exp_shape_s1a
 *  [72]       exp_shape_s3a
 *  [73]       exp_shape_s1b
 *  [74]       exp_shape_s3b
 *  [75]       exp_task_end_a (ticks)
 *  [76]       exp_task_end_b (ticks)
 *  [77]       exp_s2_end_a (ticks)
 *  [78]       exp_s2_end_b (ticks)
 *  [79]       exp_s4_start_a (ticks)
 *  [80]       exp_s4_start_b (ticks)
 *  [81]       exp_s2pf_valid_a
 *  [82]       exp_s2pf_start_a (ticks, 0 if not valid)
 *  [83]       exp_s2pf_valid_b
 *  [84]       exp_s2pf_start_b (ticks, 0 if not valid)
 *  [85]       exp_cost (ticks)
 *  [86]       exp_makespan (ticks)
 *  [87]       exp_eval_valid
 * ─────────────────────────────────────────────────────────────────────────── */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Shape constants (tick domain)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define SHAPE_A 0
#define SHAPE_B 1
#define SHAPE_C 2

static const uint32_t KTS1[3]  = {8, 4, 2};   /* S1 GEMM duration */
static const uint32_t KTS3[3]  = {4, 2, 1};   /* S3 GEMM duration */
static const uint32_t KTD1[3]  = {4, 4, 2};   /* S1 DMA duration  */
static const uint32_t KTD3[3]  = {2, 2, 1};   /* S3 DMA duration  */
static const uint32_t KMDIM[3] = {8, 4, 2};   /* M_dim            */
static const int      KBW[3]   = {1, 1, 2};   /* DMA BW: 1=64, 2=128 */

#define BW_0    0
#define BW_64   1
#define BW_128  2
#define MAX_BW  2   /* 128 B/cc ceiling */

/* RTL pf_eid encoding */
#define PF_EID_NONE   0
#define PF_EID_GHOST  1
#define PF_EID_BASE   2

/* ═══════════════════════════════════════════════════════════════════════════
 * Timing primitives (tick domain)
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint32_t best_s4_t(uint32_t r) { return (r + 1u) / 2u; }
static uint32_t best_s2_t(uint32_t r) { return best_s4_t(r) * 2u; }
static uint32_t best_task_t(uint32_t n){ return best_s2_t(n) + best_s4_t(n); }
static uint32_t best_conc_t(uint32_t n){
    return ((n + 3u) / 4u) * 6u;
}
static uint32_t umax(uint32_t a, uint32_t b){ return a > b ? a : b; }
static uint32_t umin(uint32_t a, uint32_t b){ return a < b ? a : b; }

/* ═══════════════════════════════════════════════════════════════════════════
 * snap_t (tick domain, mirrors eval_snap_t from sched_pkg.sv)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    int      valid;
    uint32_t task_start, task_end;
    uint32_t dma1_end, s1_end, s2_end, dma3_end, s3_end, s4_start;
    int      bw_s1, bw_s3;        /* BW_0 / BW_64 / BW_128 */
    int      s2pf_valid;
    uint32_t s2pf_start, s2pf_end;
    int      s2pf_bw;
    int      s4pf_valid;
    uint32_t s4pf_start;
    uint32_t ntok;
    int      pf_eid;               /* RTL encoding: 0=NONE,1=GHOST,>=2=expert */
    uint32_t pf_end;
    int      pf_full;
} snap_t;

static snap_t snap_idle(uint32_t t) {
    snap_t s;
    memset(&s, 0, sizeof(s));
    s.task_start = s.task_end  = t;
    s.dma1_end   = s.s1_end   = t;
    s.s2_end     = t;
    s.dma3_end   = s.s3_end   = s.s4_start = t;
    s.pf_eid     = PF_EID_NONE;
    return s;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * mk_snap — mirrors sched_mk_snap.sv exactly
 * ═══════════════════════════════════════════════════════════════════════════ */
static snap_t mk_snap(uint32_t start, int s1, int s3,
                      uint32_t ntok, int skip_s1, int skip_s3) {
    snap_t r = snap_idle(start);
    r.valid = 1;
    r.ntok  = ntok;

    uint32_t s1_tail = (ntok > KMDIM[s1]) ? ntok - KMDIM[s1] : 0u;
    uint32_t s3_tail = (ntok > KMDIM[s3]) ? ntok - KMDIM[s3] : 0u;

    if (skip_s1) {
        r.dma1_end = start;
        r.s1_end   = start;
        r.bw_s1    = BW_0;
        r.s2_end   = start + best_s2_t(ntok);
    } else {
        r.dma1_end = start + KTD1[s1];
        r.s1_end   = start + KTS1[s1];
        r.bw_s1    = KBW[s1];
        r.s2_end   = r.s1_end + best_s2_t(s1_tail);
    }

    if (skip_s3) {
        r.dma3_end = r.s2_end;
        r.s3_end   = r.s2_end;
        r.s4_start = r.s2_end;
        r.bw_s3    = BW_0;
        r.task_end = r.s2_end + best_s4_t(ntok);
    } else {
        r.dma3_end = r.s2_end + KTD3[s3];
        r.s3_end   = r.s2_end + KTS3[s3];
        r.s4_start = r.s3_end;
        r.bw_s3    = KBW[s3];
        r.task_end = r.s3_end + best_s4_t(s3_tail);
    }
    return r;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * pick_shapes — mirrors sched_pick_shapes.sv exactly
 * ═══════════════════════════════════════════════════════════════════════════ */
static void pick_shapes(uint32_t ntok_a, uint32_t ntok_b,
                        int sw_a, int dn_a, int sw_b, int dn_b,
                        uint32_t t0,
                        int *s1a, int *s3a, int *s1b, int *s3b) {
    /* S1 shape */
    *s1a = *s1b = (sw_a || sw_b) ? SHAPE_C : SHAPE_B;

    /* S2 end estimate for S3 decision */
    uint32_t s2a, s2b;
    if (sw_a) {
        s2a = t0 + best_s2_t(ntok_a);
    } else {
        uint32_t tail = (ntok_a > KMDIM[*s1a]) ? ntok_a - KMDIM[*s1a] : 0u;
        s2a = t0 + KTS1[*s1a] + best_s2_t(tail);
    }
    if (sw_b) {
        s2b = t0 + best_s2_t(ntok_b);
    } else {
        uint32_t tail = (ntok_b > KMDIM[*s1b]) ? ntok_b - KMDIM[*s1b] : 0u;
        s2b = t0 + KTS1[*s1b] + best_s2_t(tail);
    }

    uint32_t delta = (s2a >= s2b) ? s2a - s2b : s2b - s2a;

    /* S3 shape: kTd3[C]=1 tick; delta>=1 means S3 DMAs are time-separated */
    if (dn_a || dn_b || delta >= 1u) {
        *s3a = *s3b = SHAPE_C;
    } else {
        *s3a = *s3b = SHAPE_B;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * bw_ok — mirrors sched_bw_ok.sv exactly
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct { uint32_t lo, hi; int bw; } seg_t;

static int snap_segs(const snap_t *s, seg_t out[5], int *n) {
    *n = 0;
    int has1 = s->valid && (s->bw_s1 != BW_0) && (s->dma1_end > s->task_start);
    int has4 = s->valid && s->s2pf_valid && (s->s2pf_bw != BW_0) &&
               (s->s2pf_end > s->s2pf_start);
    int has3 = s->valid && (s->bw_s3 != BW_0) && (s->dma3_end > s->s2_end);
    int has5 = s->valid && s->s4pf_valid;

    uint32_t s1lo = s->task_start, s1hi = s->dma1_end;
    int      s1bw = s->bw_s1;
    uint32_t p4lo = s->s2pf_start, p4hi = s->s2pf_end;
    int      p4bw = s->s2pf_bw;

    if (has1 && has4 && s1lo < p4hi && p4lo < s1hi) {
        uint32_t ovl_lo = (s1lo > p4lo) ? s1lo : p4lo;
        uint32_t ovl_hi = (s1hi < p4hi) ? s1hi : p4hi;
        int merged = s1bw + p4bw;
        if (merged > MAX_BW) return 0;   /* single-snap violation */
        if (s1lo < p4lo)      out[(*n)++] = (seg_t){s1lo, p4lo, s1bw};
        else if (p4lo < s1lo) out[(*n)++] = (seg_t){p4lo, s1lo, p4bw};
        if (ovl_hi > ovl_lo)  out[(*n)++] = (seg_t){ovl_lo, ovl_hi, merged};
        if (s1hi > p4hi)      out[(*n)++] = (seg_t){p4hi, s1hi, s1bw};
        else if (p4hi > s1hi) out[(*n)++] = (seg_t){s1hi, p4hi, p4bw};
    } else {
        if (has1) out[(*n)++] = (seg_t){s1lo, s1hi, s1bw};
        if (has4) out[(*n)++] = (seg_t){p4lo, p4hi, p4bw};
    }
    if (has3) out[(*n)++] = (seg_t){s->s2_end, s->dma3_end, s->bw_s3};
    if (has5) out[(*n)++] = (seg_t){s->s4pf_start, s->s4pf_start + KTD1[SHAPE_A], BW_64};
    return 1;
}

static int bw_ok(const snap_t *a, const snap_t *b) {
    seg_t sa[5], sb[5]; int na = 0, nb = 0;
    if (!snap_segs(a, sa, &na)) return 0;
    if (!snap_segs(b, sb, &nb)) return 0;
    for (int i = 0; i < na; i++)
        for (int j = 0; j < nb; j++) {
            uint32_t lo = (sa[i].lo > sb[j].lo) ? sa[i].lo : sb[j].lo;
            uint32_t hi = (sa[i].hi < sb[j].hi) ? sa[i].hi : sb[j].hi;
            if (lo < hi && sa[i].bw + sb[j].bw > MAX_BW) return 0;
        }
    return 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * S2PF primitives — mirror sched_s2pf_pair.sv exactly
 * ═══════════════════════════════════════════════════════════════════════════ */
static int can_apply_s2pf(const snap_t *sn, int shape_s3, uint32_t pf_start) {
    uint32_t dur = KTD3[shape_s3];
    return sn->valid && (sn->bw_s3 != BW_0) &&
           (pf_start >= sn->task_start) &&
           (pf_start + dur <= sn->s2_end);
}

static snap_t apply_s2pf(snap_t sn, int shape_s3, uint32_t pf_start) {
    if (!can_apply_s2pf(&sn, shape_s3, pf_start)) return sn;
    uint32_t dur = KTD3[shape_s3];
    sn.s2pf_valid = 1;
    sn.s2pf_start = pf_start;
    sn.s2pf_end   = pf_start + dur;
    sn.s2pf_bw    = KBW[shape_s3];
    sn.dma3_end   = sn.s2_end;
    sn.s3_end     = sn.s2_end;
    sn.s4_start   = sn.s2_end;
    sn.bw_s3      = BW_0;
    sn.task_end   = sn.s2_end + best_s4_t(sn.ntok);
    return sn;
}

/* Build ca[] / ca_valid[] list (4 candidate pf_start positions for one side).
 * Returns 1 if can_a == 1 (prerequisite for having any candidates). */
static int build_s2pf_candidates(const snap_t *sn, int shape_s3,
                                  uint32_t ca[4], int ca_v[4]) {
    for (int i = 0; i < 4; i++) { ca[i] = 0; ca_v[i] = 0; }
    /* Gate: can_apply_s2pf at task_start is the prerequisite */
    if (!can_apply_s2pf(sn, shape_s3, sn->task_start)) return 0;

    uint32_t hi = sn->s2_end - KTD3[shape_s3];

    ca[0] = sn->task_start; ca_v[0] = 1;

    if (sn->dma1_end >= sn->task_start &&
        sn->dma1_end <= hi &&
        sn->dma1_end != sn->task_start) {
        ca[1] = sn->dma1_end; ca_v[1] = 1;
    }
    if (sn->s1_end >= sn->task_start &&
        sn->s1_end <= hi &&
        sn->s1_end != sn->dma1_end) {
        ca[2] = sn->s1_end; ca_v[2] = 1;
    }
    if (hi != sn->task_start) {
        ca[3] = hi; ca_v[3] = 1;
    }
    return 1;
}

/* try_s2pf_pair — mirrors the RTL's 25-combo scan with exact priority order.
 * Priority: more prefetches > fewer; ties by sum of pf_start (lower wins);
 * equal sum → first encountered in combo-index order wins. */
static void try_s2pf_pair(const snap_t *raw_a, int s3a,
                           const snap_t *raw_b, int s3b,
                           int enable, int single_latest,
                           int side_a_active, int side_b_active,
                           snap_t *out_a, snap_t *out_b, int *ok_out) {
    *out_a  = *raw_a;
    *out_b  = *raw_b;
    *ok_out = 0;

    int      best_class = -1;
    uint64_t best_sum   = UINT64_MAX;

    /* Combo 0: no prefetch (class=0, sum=0) */
    if (raw_a->valid || raw_b->valid) {
        if (bw_ok(raw_a, raw_b)) {
            *ok_out = 1;
            *out_a  = *raw_a; *out_b = *raw_b;
            best_class = 0; best_sum = 0;
        }
    }

    if (enable && side_a_active && side_b_active) {
        /* Full search: A-only, B-only, A+B */
        uint32_t ca[4], cb[4];
        int ca_v[4], cb_v[4];
        int can_a = build_s2pf_candidates(raw_a, s3a, ca, ca_v);
        int can_b = build_s2pf_candidates(raw_b, s3b, cb, cb_v);
        (void)can_a; (void)can_b; /* checked via ca_v/cb_v */

        /* Combos 1..4: A only */
        for (int ia = 0; ia < 4; ia++) {
            if (!ca_v[ia]) continue;
            snap_t ta = apply_s2pf(*raw_a, s3a, ca[ia]);
            if (bw_ok(&ta, raw_b)) {
                uint64_t ss = (uint64_t)ca[ia];
                if (!*ok_out || 1 > best_class ||
                    (1 == best_class && ss < best_sum)) {
                    *ok_out = 1;
                    *out_a = ta; *out_b = *raw_b;
                    best_class = 1; best_sum = ss;
                }
            }
        }
        /* Combos 5..8: B only */
        for (int ib = 0; ib < 4; ib++) {
            if (!cb_v[ib]) continue;
            snap_t tb = apply_s2pf(*raw_b, s3b, cb[ib]);
            if (bw_ok(raw_a, &tb)) {
                uint64_t ss = (uint64_t)cb[ib];
                if (!*ok_out || 1 > best_class ||
                    (1 == best_class && ss < best_sum)) {
                    *ok_out = 1;
                    *out_a = *raw_a; *out_b = tb;
                    best_class = 1; best_sum = ss;
                }
            }
        }
        /* Combos 9..24: A+B (outer ia, inner ib — combo index = 9+ia*4+ib) */
        for (int ia = 0; ia < 4; ia++) {
            if (!ca_v[ia]) continue;
            snap_t ta = apply_s2pf(*raw_a, s3a, ca[ia]);
            for (int ib = 0; ib < 4; ib++) {
                if (!cb_v[ib]) continue;
                snap_t tb = apply_s2pf(*raw_b, s3b, cb[ib]);
                if (bw_ok(&ta, &tb)) {
                    uint64_t ss = (uint64_t)ca[ia] + (uint64_t)cb[ib];
                    if (!*ok_out || 2 > best_class ||
                        (2 == best_class && ss < best_sum)) {
                        *ok_out = 1;
                        *out_a = ta; *out_b = tb;
                        best_class = 2; best_sum = ss;
                    }
                }
            }
        }
    } else if (enable && single_latest && side_a_active) {
        /* Combo 1 only: latest A (single_latest_only mode) */
        if (can_apply_s2pf(raw_a, s3a, raw_a->task_start)) {
            uint32_t hi_a = raw_a->s2_end - KTD3[s3a];
            snap_t ta = apply_s2pf(*raw_a, s3a, hi_a);
            if (bw_ok(&ta, raw_b)) {
                uint64_t ss = hi_a;
                if (!*ok_out || 1 > best_class ||
                    (1 == best_class && ss < best_sum)) {
                    *ok_out = 1;
                    *out_a = ta; *out_b = *raw_b;
                    best_class = 1; best_sum = ss;
                }
            }
        }
    } else if (enable && single_latest && side_b_active) {
        /* Combo 5 only: latest B (single_latest_only mode) */
        if (can_apply_s2pf(raw_b, s3b, raw_b->task_start)) {
            uint32_t hi_b = raw_b->s2_end - KTD3[s3b];
            snap_t tb = apply_s2pf(*raw_b, s3b, hi_b);
            if (bw_ok(raw_a, &tb)) {
                uint64_t ss = hi_b;
                if (!*ok_out || 1 > best_class ||
                    (1 == best_class && ss < best_sum)) {
                    *ok_out = 1;
                    *out_a = *raw_a; *out_b = tb;
                    best_class = 1; best_sum = ss;
                }
            }
        }
    }
    /* else: no additional combos; only combo 0 was tried */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * swiglu_hit / down_hit (mirrors sched_pkg.sv functions)
 * ═══════════════════════════════════════════════════════════════════════════ */
static int encode_eid(int eid_raw) { return PF_EID_BASE + eid_raw; }

static int swiglu_hit(int eid_raw, int pf_eid, uint32_t pf_end, uint32_t t) {
    if (pf_eid == PF_EID_NONE) return 0;
    if (pf_end > t) return 0;
    return (pf_eid == PF_EID_GHOST) || (pf_eid == encode_eid(eid_raw));
}
static int down_hit(int eid_raw, int pf_eid, uint32_t pf_end, int pf_full, uint32_t t) {
    return swiglu_hit(eid_raw, pf_eid, pf_end, t) && pf_full;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * score_unit (continuation_cost) — mirrors sched_score_unit.sv exactly
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint32_t score_unit(const snap_t *pa, const snap_t *pb,
                            int rem_len, int rem0_eid, uint32_t rem0_ntok,
                            uint32_t rem1_ntok,
                            uint32_t total_conc, uint32_t max_conc) {
    uint32_t INF = 0xFFFFu;   /* representable in T_W=16 bits */

    uint32_t tl = umax(pa->task_end, pb->task_end);
    uint32_t te = umin(pa->task_end, pb->task_end);

    switch (rem_len) {
    case 0:
        return tl;

    case 1: {
        /* sim1: try solo on C2 (ShapeC/B) and C3 (ShapeC/B), plus split */
        int sw_c2 = swiglu_hit(rem0_eid, pa->pf_eid, pa->pf_end, tl);
        int dn_c2 = down_hit  (rem0_eid, pa->pf_eid, pa->pf_end, pa->pf_full, tl);
        int sw_c3 = swiglu_hit(rem0_eid, pb->pf_eid, pb->pf_end, tl);
        int dn_c3 = down_hit  (rem0_eid, pb->pf_eid, pb->pf_end, pb->pf_full, tl);

        snap_t c2c = mk_snap(tl, SHAPE_C, SHAPE_C, rem0_ntok, sw_c2, dn_c2);
        snap_t c2b = mk_snap(tl, SHAPE_B, SHAPE_B, rem0_ntok, 0,     0);
        snap_t c3c = mk_snap(tl, SHAPE_C, SHAPE_C, rem0_ntok, sw_c3, dn_c3);
        snap_t c3b = mk_snap(tl, SHAPE_B, SHAPE_B, rem0_ntok, 0,     0);

        uint32_t cost = c2c.task_end;
        if (!sw_c2 && c2b.task_end < cost) cost = c2b.task_end;
        if (c3c.task_end < cost) cost = c3c.task_end;
        if (!sw_c3 && c3b.task_end < cost) cost = c3b.task_end;

        /* Split candidate (only if rem0_ntok >= 2) */
        if (rem0_ntok >= 2u) {
            uint32_t sa_ntok = (rem0_ntok + 1u) / 2u;
            uint32_t sb_ntok = rem0_ntok - sa_ntok;
            int s1a, s3a, s1b, s3b;
            pick_shapes(sa_ntok, sb_ntok, sw_c2, dn_c2, sw_c3, dn_c3, tl,
                        &s1a, &s3a, &s1b, &s3b);
            snap_t ra = mk_snap(tl, s1a, s3a, sa_ntok, sw_c2, dn_c2);
            snap_t rb = mk_snap(tl, s1b, s3b, sb_ntok, sw_c3, dn_c3);
            snap_t oa, ob; int split_ok;
            try_s2pf_pair(&ra, s3a, &rb, s3b,
                          /*enable=*/1, /*single_latest=*/0,
                          /*side_a=*/1, /*side_b=*/1,
                          &oa, &ob, &split_ok);
            if (split_ok) {
                uint32_t ms = umax(oa.task_end, ob.task_end);
                if (ms < cost) cost = ms;
            }
        }
        return cost;
    }

    case 2: {
        /* Closed-form min(parallel, serial) */
        uint32_t bc0 = best_conc_t(rem0_ntok);
        uint32_t bc1 = best_conc_t(rem1_ntok);
        uint32_t pc   = tl + umax(bc0, bc1);
        uint32_t ser  = te + best_task_t(rem0_ntok) + best_task_t(rem1_ntok);
        uint32_t serc = umax(ser, tl);
        return umin(pc, serc);
    }

    default: {
        /* Greedy heuristic */
        uint32_t half_sum = total_conc / 2u;   /* total_conc >> 1 */
        uint32_t extra    = umax(max_conc, half_sum);
        return tl + extra;
    }
    }
    return INF;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * eval_lane — full golden model; produces all expected output fields
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    int      bw_ok;
    int      s1a, s3a, s1b, s3b;
    snap_t   snap_a, snap_b;
    uint32_t cost;
    uint32_t makespan;
    int      eval_valid;
} eval_result_t;

static eval_result_t eval_lane(
    /* control */
    int cand_valid, int enable_s2pf, int single_latest,
    int force_a, int forced_s1a, int forced_s3a,
    int force_b, int forced_s1b, int forced_s3b,
    /* task A */
    int      side_a_valid,
    uint32_t start_a, uint32_t ntok_a, int sw_a, int dn_a,
    /* task B */
    int      side_b_valid,
    uint32_t start_b, uint32_t ntok_b, int sw_b, int dn_b,
    /* pick_shapes t0 */
    uint32_t shape_t0,
    /* base snaps (used when side_x_valid=0) */
    snap_t base_a, snap_t base_b,
    /* rem info for score */
    int rem_len, int rem0_eid, uint32_t rem0_ntok, uint32_t rem1_ntok,
    uint32_t total_conc, uint32_t max_conc)
{
    eval_result_t r;
    memset(&r, 0, sizeof(r));

    /* 1. pick_shapes */
    int ps1a, ps3a, ps1b, ps3b;
    pick_shapes(ntok_a, ntok_b, sw_a, dn_a, sw_b, dn_b, shape_t0,
                &ps1a, &ps3a, &ps1b, &ps3b);
    r.s1a = force_a ? forced_s1a : ps1a;
    r.s3a = force_a ? forced_s3a : ps3a;
    r.s1b = force_b ? forced_s1b : ps1b;
    r.s3b = force_b ? forced_s3b : ps3b;

    /* 2. mk_snap A/B */
    snap_t raw_a = base_a;
    if (side_a_valid) {
        raw_a = mk_snap(start_a, r.s1a, r.s3a, ntok_a, sw_a, dn_a);
    }
    snap_t raw_b = base_b;
    if (side_b_valid) {
        raw_b = mk_snap(start_b, r.s1b, r.s3b, ntok_b, sw_b, dn_b);
    }

    /* 3. try_s2pf_pair */
    snap_t out_a, out_b; int ok;
    try_s2pf_pair(&raw_a, r.s3a, &raw_b, r.s3b,
                  enable_s2pf, single_latest,
                  side_a_valid, side_b_valid,
                  &out_a, &out_b, &ok);

    r.bw_ok  = ok;
    r.snap_a = out_a;
    r.snap_b = out_b;

    /* 4. makespan */
    r.makespan = umax(out_a.task_end, out_b.task_end);

    /* 5. score_unit (continuation_cost) */
    r.cost = score_unit(&out_a, &out_b,
                        rem_len, rem0_eid, rem0_ntok, rem1_ntok,
                        total_conc, max_conc);

    /* 6. eval_valid */
    r.eval_valid = cand_valid && ok && (side_a_valid || side_b_valid);

    return r;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Output helper
 * ═══════════════════════════════════════════════════════════════════════════ */
static void print_snap_fields(const snap_t *s) {
    printf(" %d %u %u %u %u %u %u %u %u %d %d %d %u %u %d %u %d %u %d",
           s->valid,
           s->task_start, s->task_end,
           s->dma1_end, s->s1_end, s->s2_end,
           s->dma3_end, s->s3_end, s->s4_start,
           s->bw_s1, s->bw_s3,
           s->s2pf_valid, s->s2pf_start, s->s2pf_end, s->s2pf_bw,
           s->ntok,
           s->pf_eid, s->pf_end,
           s->pf_full);
}

/* Emit one test vector line.  cand_id used only for debug comments. */
static void emit(int cand_id,
                 /* control */
                 int plan_type, int cluster_a,
                 int enable_s2pf, int single_latest,
                 int force_a, int fs1a, int fs3a,
                 int force_b, int fs1b, int fs3b,
                 int cost_only_tie,
                 /* tasks */
                 int      side_a_valid, int      side_b_valid,
                 uint32_t start_a,      uint32_t start_b,
                 int      eid_a,        int      eid_b,
                 uint32_t ntok_a,       uint32_t ntok_b,
                 uint32_t tok_start_a,  uint32_t tok_start_b,
                 int sw_a, int dn_a, int sw_b, int dn_b,
                 uint32_t shape_t0,
                 /* rem */
                 int rem_len, int rem0_eid, uint32_t rem0_ntok, uint32_t rem1_ntok,
                 uint32_t total_conc, uint32_t max_conc,
                 /* base snaps */
                 snap_t base_a, snap_t base_b) {

    eval_result_t res = eval_lane(
        /*cand_valid=*/1, enable_s2pf, single_latest,
        force_a, fs1a, fs3a, force_b, fs1b, fs3b,
        side_a_valid, start_a, ntok_a, sw_a, dn_a,
        side_b_valid, start_b, ntok_b, sw_b, dn_b,
        shape_t0, base_a, base_b,
        rem_len, rem0_eid, rem0_ntok, rem1_ntok,
        total_conc, max_conc);

    /* Fields [0..31] */
    printf("%d %d %d %d %d %d %d %d %d %d %d",
           plan_type, cluster_a,
           enable_s2pf, single_latest,
           force_a, force_b, fs1a, fs3a, fs1b, fs3b,
           cost_only_tie);
    printf(" %d %d %u %u %d %d %u %u %u %u",
           side_a_valid, side_b_valid,
           start_a, start_b,
           eid_a, eid_b,
           ntok_a, ntok_b,
           tok_start_a, tok_start_b);
    printf(" %d %d %d %d %u",
           sw_a, dn_a, sw_b, dn_b, shape_t0);
    printf(" %d %d %u %u %u %u",
           rem_len, rem0_eid, rem0_ntok, rem1_ntok,
           total_conc, max_conc);

    /* Fields [32..50] base_snap_a */
    print_snap_fields(&base_a);

    /* Fields [51..69] base_snap_b */
    print_snap_fields(&base_b);

    /* Fields [70..87] expected outputs */
    printf(" %d %d %d %d %d",
           res.bw_ok, res.s1a, res.s3a, res.s1b, res.s3b);
    printf(" %u %u %u %u %u %u",
           res.snap_a.task_end, res.snap_b.task_end,
           res.snap_a.s2_end,   res.snap_b.s2_end,
           res.snap_a.s4_start, res.snap_b.s4_start);
    printf(" %d %u %d %u",
           res.snap_a.s2pf_valid, res.snap_a.s2pf_valid ? res.snap_a.s2pf_start : 0u,
           res.snap_b.s2pf_valid, res.snap_b.s2pf_valid ? res.snap_b.s2pf_start : 0u);
    printf(" %u %u %d",
           res.cost, res.makespan, res.eval_valid);
    printf("\n");
    (void)cand_id;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test case generation
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void) {
    snap_t idle0 = snap_idle(0);

    int tid = 0;

    /* ─── Group 1: PAIR basic — pick_shapes and mk_snap timing ──────────── */
    /* No cache hits, varying ntok; enable_s2pf=0, rem_len=0 */
    {
        static const uint32_t ntoks[] = {1, 2, 4, 8, 12, 16, 32, 64};
        for (int ia = 0; ia < 8; ia++) {
            for (int ib = 0; ib < 8; ib++) {
                uint32_t na = ntoks[ia], nb = ntoks[ib];
                emit(tid++, 0,0,0,0, 0,0,0,0,0,0,0,
                     1,1, 0,0, 0,1, na,nb, 0,0,
                     0,0,0,0, 0,  /* t0=0 */
                     0,0,1,1, 0,0,  /* rem_len=0, rem0_ntok=1 */
                     idle0, idle0);
            }
        }
    }

    /* ─── Group 2: Cache hit combinations ───────────────────────────────── */
    /* sw_a, dn_a, sw_b, dn_b combinations; ntok=8 */
    {
        /* (sw,dn) valid pairs: (0,0),(1,0),(1,1).
         * dn=1 implies sw=1 (down_hit requires swiglu_hit first) */
        int sw_dn_pairs[3][2] = {{0,0},{1,0},{1,1}};
        uint32_t na = 8, nb = 8;
        for (int ia = 0; ia < 3; ia++) {
            for (int ib = 0; ib < 3; ib++) {
                int swa = sw_dn_pairs[ia][0], dna = sw_dn_pairs[ia][1];
                int swb = sw_dn_pairs[ib][0], dnb = sw_dn_pairs[ib][1];
                emit(tid++, 0,0,0,0, 0,0,0,0,0,0,0,
                     1,1, 0,0, 0,1, na,nb, 0,0,
                     swa,dna,swb,dnb, 0,
                     0,0,4,4, 0,0,
                     idle0, idle0);
            }
        }
    }

    /* ─── Group 3: Non-zero start_a/start_b (staggered scheduling) ──────── */
    {
        uint32_t na = 8, nb = 4;
        uint32_t starts_a[] = {0, 4, 8};
        uint32_t starts_b[] = {0, 2, 6};
        for (int ia = 0; ia < 3; ia++) {
            for (int ib = 0; ib < 3; ib++) {
                emit(tid++, 0,0,0,0, 0,0,0,0,0,0,0,
                     1,1, starts_a[ia], starts_b[ib], 0,1, na,nb, 0,0,
                     0,0,0,0, umax(starts_a[ia], starts_b[ib]),
                     0,0,1,1, 0,0,
                     idle0, idle0);
            }
        }
    }

    /* ─── Group 4: Forced shapes (bypass pick_shapes) ───────────────────── */
    {
        /* Force both sides to ShapeC — this will cause bw_ok=0 (256 B/cc S1) */
        uint32_t na = 4, nb = 4;
        /* forced ShapeC+ShapeC: S1 DMA overlap → 128+128=256 > 128 → fail */
        emit(tid++, 0,0,0,0, 1,SHAPE_C,SHAPE_C,1,SHAPE_C,SHAPE_C,0,
             1,1, 0,0, 0,1, na,nb, 0,0,
             0,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);

        /* Force A=ShapeC, B=ShapeB: 128+64=192 > 128 → fail */
        emit(tid++, 0,0,0,0, 1,SHAPE_C,SHAPE_C,1,SHAPE_B,SHAPE_B,0,
             1,1, 0,0, 0,1, na,nb, 0,0,
             0,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);

        /* Force A=ShapeB, B=ShapeB: 64+64=128 → ok */
        emit(tid++, 0,0,0,0, 1,SHAPE_B,SHAPE_B,1,SHAPE_B,SHAPE_B,0,
             1,1, 0,0, 0,1, na,nb, 0,0,
             0,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);

        /* Force ShapeA+ShapeA: 64+64=128 → ok (kTd1[A]=4) */
        emit(tid++, 0,0,0,0, 1,SHAPE_A,SHAPE_B,1,SHAPE_A,SHAPE_B,0,
             1,1, 0,0, 0,1, na,nb, 0,0,
             0,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);
    }

    /* ─── Group 5: S2PF enabled — both sides can prefetch ───────────────── */
    {
        /* Standard PAIR: ntok_a=ntok_b=8, ShapeB S1+S3, enable_s2pf=1 */
        uint32_t ntoks[] = {4, 8, 16, 32};
        for (int i = 0; i < 4; i++) {
            uint32_t n = ntoks[i];
            emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
                 1,1, 0,0, 0,1, n,n, 0,0,
                 0,0,0,0, 0,
                 0,0,1,1, 0,0,
                 idle0, idle0);
        }
    }

    /* ─── Group 6: S2PF enabled — one side has skip_s3 (bw_s3=0 → no s2pf) */
    {
        /* Side A has dn_a=1 → skip_s3 → bw_s3=0 → no s2pf on A */
        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,1, 8,8, 0,0,
             1,1, 0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);

        /* Side B has dn_b=1 → skip_s3 → bw_s3=0 → no s2pf on B */
        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,1, 8,8, 0,0,
             0,0, 1,1, 0,
             0,0,1,1, 0,0,
             idle0, idle0);

        /* Both dn → no s2pf anywhere */
        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,1, 8,8, 0,0,
             1,1, 1,1, 0,
             0,0,1,1, 0,0,
             idle0, idle0);
    }

    /* ─── Group 7: S2PF enabled — sw hit makes ShapeC, larger S2 window ── */
    {
        /* sw_a=1: ShapeC S1, skip_s1 → big S2 window; s2pf possible */
        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,1, 8,8, 0,0,
             1,0, 0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);

        /* sw_a=1, sw_b=1: both skip_s1, large S2 windows */
        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,1, 8,8, 0,0,
             1,0, 1,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);

        /* ntok=2: small token count, s2 window tight */
        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,1, 2,2, 0,0,
             0,0, 0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);
    }

    /* ─── Group 8: single_latest_only mode (SOLO / not_both_idle) ────────── */
    {
        uint32_t ntoks[] = {4, 8, 16};
        for (int i = 0; i < 3; i++) {
            uint32_t n = ntoks[i];
            /* A active, single_latest=1: only try latest A pf_start */
            emit(tid++, 2,1,1,1, 0,0,0,0,0,0,0,
                 1,0, 0,0, 0,PF_EID_NONE, n,0, 0,0,
                 0,0, 0,0, 0,
                 0,0,1,1, 0,0,
                 idle0, idle0);

            /* B active, single_latest=1 */
            emit(tid++, 2,1,1,1, 0,0,0,0,0,0,0,
                 0,1, 0,0, PF_EID_NONE,0, 0,n, 0,0,
                 0,0, 0,0, 0,
                 0,0,1,1, 0,0,
                 idle0, idle0);
        }
    }

    /* ─── Group 9: SOLO — one side has a busy base_snap ─────────────────── */
    {
        /* Build a busy base snap: expert 0, ntok=4, ShapeB, start=0 */
        snap_t busy = mk_snap(0, SHAPE_B, SHAPE_B, 4, 0, 0);
        busy.valid   = 1;
        busy.pf_eid  = PF_EID_NONE;
        busy.pf_full = 0;

        /* SOLO B: side_a=0 (use busy base_a), side_b=1 (new task) */
        emit(tid++, 2,1,1,1, 0,0,0,0,0,0,0,
             0,1, 0,0, 0,0, 0,8, 0,0,
             0,0, 0,0, 0,
             0,0,1,1, 0,0,
             busy, idle0);

        emit(tid++, 2,1,1,1, 0,0,0,0,0,0,0,
             0,1, 0,0, 0,1, 0,16, 0,0,
             0,0, 0,0, 0,
             0,0,1,1, 0,0,
             busy, idle0);

        /* SOLO A: side_a=1 (new), side_b=0 (use busy base_b) */
        emit(tid++, 2,0,1,1, 0,0,0,0,0,0,0,
             1,0, 0,0, 0,0, 8,0, 0,0,
             0,0, 0,0, 0,
             0,0,1,1, 0,0,
             idle0, busy);

        /* SOLO with sw hit on new task (skip_s1) */
        emit(tid++, 2,1,1,1, 0,0,0,0,0,0,0,
             0,1, 0,0, 0,1, 0,8, 0,0,
             0,0, 1,0, 0,
             0,0,1,1, 0,0,
             busy, idle0);
    }

    /* ─── Group 10: score_unit rem_len=1 (no cache, simple sim1) ─────────── */
    {
        /* rem_len=1, rem0_ntok=1: only solo paths (no split, ntok<2) */
        uint32_t ntoks_pair[] = {4, 8};
        uint32_t rem0s[] = {1, 4, 8};
        for (int ip = 0; ip < 2; ip++) {
            for (int ir = 0; ir < 3; ir++) {
                uint32_t n = ntoks_pair[ip];
                uint32_t r = rem0s[ir];
                emit(tid++, 0,0,0,0, 0,0,0,0,0,0,0,
                     1,1, 0,0, 0,1, n,n, 0,0,
                     0,0,0,0, 0,
                     1, 0, r, 0, 0, 0,
                     idle0, idle0);
            }
        }
    }

    /* ─── Group 11: score_unit rem_len=2 ────────────────────────────────── */
    {
        uint32_t n0s[] = {4, 8, 16};
        uint32_t n1s[] = {2, 4,  8};
        for (int i = 0; i < 3; i++) {
            emit(tid++, 0,0,0,0, 0,0,0,0,0,0,0,
                 1,1, 0,0, 0,1, 8,8, 0,0,
                 0,0,0,0, 0,
                 2, 0, n0s[i], n1s[i], 0, 0,
                 idle0, idle0);
        }
    }

    /* ─── Group 12: score_unit rem_len>=3 (greedy path) ──────────────────── */
    {
        /* total_conc and max_conc determine the cost */
        struct { uint32_t tot; uint32_t mx; } cases[] = {
            {12, 8}, {8, 6}, {20, 12}, {6, 4}
        };
        for (int i = 0; i < 4; i++) {
            emit(tid++, 0,0,0,0, 0,0,0,0,0,0,0,
                 1,1, 0,0, 0,1, 8,8, 0,0,
                 0,0,0,0, 0,
                 3, 0, 4, 4, cases[i].tot, cases[i].mx,
                 idle0, idle0);
        }
    }

    /* ─── Group 13: S2PF cross-BW violation — forced overlap ─────────────── */
    {
        /* Force ShapeC+ShapeC; both sides: S1 DMA = 128+128=256 → bw_ok=0.
         * Even with enable_s2pf=1, the no-pf baseline fails, and s2pf can't fix it. */
        emit(tid++, 0,0,1,0, 1,SHAPE_C,SHAPE_C,1,SHAPE_C,SHAPE_C,0,
             1,1, 0,0, 0,1, 8,8, 0,0,
             0,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);
    }

    /* ─── Group 14: SPLIT plan_type (tok_start non-zero) ─────────────────── */
    {
        /* SPLIT: same expert, split ntok; plan_type=1 */
        emit(tid++, 1,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,0, 8,8, 0,8,
             0,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);

        emit(tid++, 1,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,0, 16,16, 0,16,
             0,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);
    }

    /* ─── Group 15: ntok=1 edge case (tail=0 → skip_s2, skip_s4) ─────────── */
    {
        /* ntok=1 with ShapeB: s1_tail=0 → skip_s2; s3_tail=0 → skip_s4 */
        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,1, 1,1, 0,0,
             0,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);

        /* ntok=1 with sw_a=1: S1 skipped, ntok goes entirely to S2 */
        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,0, 0,1, 1,1, 0,0,
             1,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);
    }

    /* ─── Group 16: cost_only_tie=1 (snap_min forced to 0 in score_key) ──── */
    {
        emit(tid++, 0,0,0,0, 0,0,0,0,0,0,1,
             1,1, 0,0, 0,1, 8,4, 0,0,
             0,0,0,0, 0,
             0,0,1,1, 0,0,
             idle0, idle0);
    }

    /* ─── Group 17: S2PF — asymmetric ntok (different s2 windows) ──────────*/
    {
        uint32_t na_vals[] = {4, 8, 16, 32};
        uint32_t nb_vals[] = {16, 4, 8,  4};
        for (int i = 0; i < 4; i++) {
            emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
                 1,1, 0,0, 0,1, na_vals[i], nb_vals[i], 0,0,
                 0,0,0,0, 0,
                 0,0,1,1, 0,0,
                 idle0, idle0);
        }
    }

    /* ─── Group 18: staggered start + s2pf ──────────────────────────────── */
    {
        /* B starts later → their S1 DMA segments don't overlap for ShapeC */
        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 0,4, 0,1, 8,8, 0,0,
             0,0,0,0, 4,
             0,0,1,1, 0,0,
             idle0, idle0);

        emit(tid++, 0,0,1,0, 0,0,0,0,0,0,0,
             1,1, 2,0, 0,1, 8,8, 0,0,
             0,0,0,0, 2,
             0,0,1,1, 0,0,
             idle0, idle0);
    }

    fprintf(stderr, "Generated %d test vectors.\n", tid);
    return 0;
}
