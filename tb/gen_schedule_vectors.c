/* gen_schedule_vectors.c
 * --------------------------------------------------------------------------
 * End-to-end golden-vector generator for sched_schedule_core.
 *
 * The generated file contains:
 *   - one full request per test
 *   - the stable ntokens-descending rem list that a CVA6-side driver should
 *     expose to the RTL as top4 heads
 *   - the compact plan emitted by moe_make_hw_plan(), which mirrors RTL
 *     commit_unit output.
 */
#include "moe_scheduler.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint16_t eid;
    uint16_t ntokens;
} rem_item_t;

static uint32_t rng_state = 0x12345678u;

static uint32_t rnd_u32(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

static uint32_t best_conc_ticks(uint32_t n)
{
    return ((n + 3u) / 4u) * 6u;
}

static void make_request(moe_request_t *req, uint16_t n, int pattern)
{
    memset(req, 0, sizeof(*req));
    req->n_experts = n;
    for (uint16_t i = 0; i < n; i++) {
        req->experts[i].expert_id = i;
        if (pattern == 0) {
            req->experts[i].ntokens = (uint16_t)(1u + (i % 32u));
        } else if (pattern == 1) {
            req->experts[i].ntokens = (uint16_t)(1u + ((n - i) % 64u));
        } else if (pattern == 2) {
            /* Lots of tiny tails to exercise EXACT_TAIL_MAX behavior. */
            req->experts[i].ntokens = (uint16_t)(1u + ((i + n) % 4u));
        } else {
            req->experts[i].ntokens = (uint16_t)(1u + (rnd_u32() % 128u));
        }
    }

    switch ((rnd_u32() + (uint32_t)pattern) % 5u) {
    case 0:
        req->cache_eid_c2 = -1;
        req->cache_eid_c3 = -1;
        break;
    case 1:
        req->cache_eid_c2 = (int16_t)(rnd_u32() % n);
        req->cache_eid_c3 = -1;
        break;
    case 2:
        req->cache_eid_c2 = -1;
        req->cache_eid_c3 = (int16_t)(rnd_u32() % n);
        break;
    default:
        req->cache_eid_c2 = (int16_t)(rnd_u32() % n);
        req->cache_eid_c3 = (int16_t)(rnd_u32() % n);
        break;
    }
}

static void make_sorted_rem(const moe_request_t *req, rem_item_t *rem)
{
    for (uint16_t i = 0; i < req->n_experts; i++) {
        rem[i].eid = req->experts[i].expert_id;
        rem[i].ntokens = req->experts[i].ntokens;
    }

    for (uint16_t i = 1; i < req->n_experts; i++) {
        rem_item_t key = rem[i];
        int j = (int)i - 1;
        while (j >= 0 && rem[j].ntokens < key.ntokens) {
            rem[j + 1] = rem[j];
            j--;
        }
        rem[j + 1] = key;
    }
}

static void emit_entry(const moe_hw_plan_entry_t *e)
{
    printf("%u %u %u %u %u %u %u %u %u %u %u\n",
           (unsigned)e->valid,
           (unsigned)e->desc.cluster,
           (unsigned)e->desc.expert_id,
           (unsigned)e->desc.token_start_rank,
           (unsigned)e->desc.ntokens,
           (unsigned)e->desc.shape_s1,
           (unsigned)e->desc.shape_s3,
           (unsigned)e->desc.skip_s1,
           (unsigned)e->desc.skip_s3,
           (unsigned)e->desc.has_s2pf,
           (unsigned)e->allow_s4pf);
}

int main(void)
{
    enum { PATTERNS_PER_N = 8 };
    const unsigned total_tests = MOE_MAX_EXPERTS * PATTERNS_PER_N;

    printf("%u\n", total_tests);

    for (uint16_t n = 1; n <= MOE_MAX_EXPERTS; n++) {
        for (int p = 0; p < PATTERNS_PER_N; p++) {
            moe_request_t req;
            rem_item_t rem[MOE_MAX_EXPERTS];
            moe_hw_plan_entry_t plan[MOE_MAX_TASKS];
            uint16_t n_plan = 0;
            moe_status_t st;

            make_request(&req, n, p);
            make_sorted_rem(&req, rem);
            st = moe_make_hw_plan(&req, plan, &n_plan);
            if (st != MOE_OK) {
                fprintf(stderr, "moe_make_hw_plan failed n=%u p=%d st=%d\n",
                        (unsigned)n, p, st);
                return 1;
            }

            printf("%u %u %d %d %u\n",
                   (unsigned)((n - 1u) * PATTERNS_PER_N + (uint16_t)p),
                   (unsigned)n,
                   (int)req.cache_eid_c2,
                   (int)req.cache_eid_c3,
                   (unsigned)n_plan);

            for (uint16_t i = 0; i < n; i++) {
                printf("%u %u %u\n",
                       (unsigned)rem[i].eid,
                       (unsigned)rem[i].ntokens,
                       (unsigned)best_conc_ticks(rem[i].ntokens));
            }

            for (uint16_t i = 0; i < n_plan; i++) {
                emit_entry(&plan[i]);
            }
        }
    }

    return 0;
}
