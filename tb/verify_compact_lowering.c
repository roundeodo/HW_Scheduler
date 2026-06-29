/* verify_compact_lowering.c
 * --------------------------------------------------------------------------
 * Software equivalence check for the RTL compact-plan lowering path.
 *
 * It compares three paths:
 *   1. legacy timing lower_plan()     (test-only entry)
 *   2. public moe_schedule()          (now compact-plan lowering)
 *   3. moe_make_hw_plan()+moe_lower_hw_plan()
 *
 * This does not run RTL.  It proves that replacing timing/snap-dependent
 * lowering with desc+allow_s4pf lowering does not change software-visible
 * tasks[]/dma_ops[] for the tested requests.
 */
#include "moe_scheduler.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef MOE_SCHEDULER_ENABLE_LEGACY_CHECK
extern moe_status_t moe_schedule_legacy_timing(const moe_request_t *req,
                                               moe_schedule_t *out);
#endif

static uint32_t rng_state = 0x12345678u;

static uint32_t rnd_u32(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

static int same_task(const moe_task_t *a, const moe_task_t *b)
{
    return a->cluster == b->cluster &&
           a->expert_id == b->expert_id &&
           a->token_start_rank == b->token_start_rank &&
           a->ntokens == b->ntokens &&
           a->shape_s1 == b->shape_s1 &&
           a->shape_s3 == b->shape_s3 &&
           a->dma_s1 == b->dma_s1 &&
           a->dma_s3 == b->dma_s3 &&
           a->skip_s1 == b->skip_s1 &&
           a->skip_s3 == b->skip_s3 &&
           a->skip_s2 == b->skip_s2 &&
           a->skip_s4 == b->skip_s4 &&
           a->m_s2_exec == b->m_s2_exec &&
           a->m_s4_exec == b->m_s4_exec;
}

static int same_dma(const moe_dma_op_t *a, const moe_dma_op_t *b)
{
    return a->task_idx == b->task_idx &&
           a->kind == b->kind &&
           a->dma == b->dma &&
           a->expert_id == b->expert_id;
}

static int same_schedule(const moe_schedule_t *a, const moe_schedule_t *b,
                         const char *tag, int tid)
{
    if (a->n_tasks != b->n_tasks) {
        printf("[FAIL] tid=%d %s n_tasks got=%u exp=%u\n",
               tid, tag, (unsigned)a->n_tasks, (unsigned)b->n_tasks);
        return 0;
    }
    if (a->n_dma_ops != b->n_dma_ops) {
        printf("[FAIL] tid=%d %s n_dma_ops got=%u exp=%u\n",
               tid, tag, (unsigned)a->n_dma_ops, (unsigned)b->n_dma_ops);
        return 0;
    }
    for (uint16_t i=0;i<a->n_tasks;i++) {
        if (!same_task(&a->tasks[i], &b->tasks[i])) {
            printf("[FAIL] tid=%d %s task[%u] mismatch\n", tid, tag, (unsigned)i);
            return 0;
        }
    }
    for (uint16_t i=0;i<a->n_dma_ops;i++) {
        if (!same_dma(&a->dma_ops[i], &b->dma_ops[i])) {
            printf("[FAIL] tid=%d %s dma[%u] mismatch\n", tid, tag, (unsigned)i);
            return 0;
        }
    }
    return 1;
}

static void make_request(moe_request_t *req, uint16_t n, int pattern)
{
    memset(req, 0, sizeof(*req));
    req->n_experts = n;
    for (uint16_t i=0;i<n;i++) {
        req->experts[i].expert_id = i;
        if (pattern == 0) {
            req->experts[i].ntokens = (uint16_t)(1u + (i % 32u));
        } else if (pattern == 1) {
            req->experts[i].ntokens = (uint16_t)(1u + ((n - i) % 64u));
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

int main(void)
{
#ifndef MOE_SCHEDULER_ENABLE_LEGACY_CHECK
    printf("[FAIL] compile with -DMOE_SCHEDULER_ENABLE_LEGACY_CHECK\n");
    return 1;
#else
    int total = 0;
    int fail = 0;

    for (uint16_t n=1;n<=MOE_MAX_EXPERTS;n++) {
        for (int p=0;p<8;p++) {
            moe_request_t req;
            moe_schedule_t legacy_sch, public_sch, compact_sch;
            moe_hw_plan_entry_t hw_plan[MOE_MAX_TASKS];
            uint16_t n_plan = 0;

            make_request(&req, n, p);

            moe_status_t st0 = moe_schedule_legacy_timing(&req, &legacy_sch);
            moe_status_t st1 = moe_schedule(&req, &public_sch);
            moe_status_t st2 = moe_make_hw_plan(&req, hw_plan, &n_plan);
            moe_status_t st3 = (st2 == MOE_OK) ?
                moe_lower_hw_plan(&req, hw_plan, n_plan, &compact_sch) : st2;

            total++;
            if (st0 != MOE_OK || st1 != MOE_OK || st2 != MOE_OK || st3 != MOE_OK) {
                printf("[FAIL] tid=%d status legacy=%d public=%d make=%d lower=%d\n",
                       total, st0, st1, st2, st3);
                fail++;
                continue;
            }
            if (!same_schedule(&public_sch, &legacy_sch, "public-vs-legacy", total) ||
                !same_schedule(&compact_sch, &legacy_sch, "compact-vs-legacy", total)) {
                fail++;
            }
        }
    }

    printf("compact lowering equivalence: total=%d pass=%d fail=%d\n",
           total, total - fail, fail);
    return fail ? 1 : 0;
#endif
}
