#!/usr/bin/env python3
"""Generate per-round winner-token traces from the current policy model."""

from __future__ import annotations

import random
import sys
from pathlib import Path


TICK_CC = 11_264
N_TESTS = 512

IDEA_MODEL = Path(__file__).resolve().parents[2] / "Idea_Model"
sys.path.insert(0, str(IDEA_MODEL))

import scheduler_hw_fixed_policy as model  # noqa: E402


SINGLE_SHAPES = tuple(model.hw.N1_PRUNED_SOLO_SHAPES)


def choose_transition(state: model.PolicyState) -> model.Transition:
    transitions = model.generate_one_idle_shape_successors(
        state, policy="balanced", top_policy="pruned", n1_policy="pruned"
    )
    if len(state.remaining) == 1 or state.c2.task_end != state.c3.task_end:
        return min(
            transitions,
            key=lambda tr: max(tr.state.c2.task_end, tr.state.c3.task_end),
        )
    return min(
        transitions,
        key=lambda tr: (
            model.hw_v2_continuation(
                tr.state.c2, tr.state.c3, tr.state.remaining, policy="balanced"
            ),
            len(tr.state.remaining),
            max(tr.state.c2.task_end, tr.state.c3.task_end),
        ),
    )


def token_from_tag(state: model.PolicyState, tag: str) -> tuple[int, int]:
    if len(state.remaining) == 1:
        if tag.startswith("last_solo_c"):
            cluster = int(tag[len("last_solo_c")])
            shape_code = tag.rsplit("_", 1)[1]
            shape = (int(shape_code[0]), int(shape_code[1]))
            return 0, (0 if cluster == 2 else len(SINGLE_SHAPES)) + SINGLE_SHAPES.index(shape)
        if tag.startswith("last_split_"):
            return 0, 10
        if tag.startswith("last_release_c"):
            release_index = int(tag.rsplit("_", 1)[1])
            return 0, 11 + release_index
        raise RuntimeError(f"unmapped LAST_EXPERT tag {tag}")

    if state.c2.task_end == state.c3.task_end:
        if tag == "pair_0_1":
            return 1, 0
        if tag == "pair_1_2":
            return 1, 1
        if tag == "pair_2_3":
            return 1, 2
        if tag.startswith("split_0_"):
            cut = int(tag.rsplit("_", 1)[1])
            half = (state.remaining[0][1] + 1) // 2
            return 1, 3 if cut == half else 4
        raise RuntimeError(f"unmapped BOTH_IDLE tag {tag}")

    if tag.startswith("one_idle_adaptive_c"):
        return 2, 3 + int(tag.rsplit("p", 1)[1])
    if tag.startswith("one_idle_c"):
        return 2, int(tag.rsplit("p", 1)[1])
    raise RuntimeError(f"unmapped ONE_IDLE tag {tag}")


def make_case(rng: random.Random, tid: int) -> tuple[dict[int, int], int, int]:
    n = tid // 8 + 1
    pattern = tid % 8
    if pattern == 0:
        ntoks = [1 + (i % 32) for i in range(n)]
    elif pattern == 1:
        ntoks = [1 + ((n - i) % 64) for i in range(n)]
    elif pattern == 2:
        ntoks = [1 + ((i + n) % 4) for i in range(n)]
    else:
        ntoks = [rng.randrange(1, 129) for _ in range(n)]
    token_dist = {eid: ntok for eid, ntok in enumerate(ntoks)}

    cache_kind = rng.randrange(5)
    if cache_kind == 0:
        cache2, cache3 = -1, -1
    elif cache_kind == 1:
        cache2, cache3 = rng.randrange(n), -1
    elif cache_kind == 2:
        cache2, cache3 = -1, rng.randrange(n)
    else:
        cache2, cache3 = rng.randrange(n), rng.randrange(n)
    return token_dist, cache2, cache3


def main() -> None:
    rng = random.Random(0xC0DE_6006)
    print(N_TESTS)
    for tid in range(N_TESTS):
        token_dist, cache2, cache3 = make_case(rng, tid)
        state = model.initial_state(token_dist, cache2, cache3)
        sorted_remaining = state.remaining
        trace: list[tuple[int, int]] = []
        while state.remaining:
            chosen = choose_transition(state)
            trace.append(token_from_tag(state, chosen.tag))
            state = chosen.state

        final_cycles = model.terminal_cost(state)
        assert final_cycles % TICK_CC == 0
        print(tid, len(sorted_remaining), cache2, cache3, len(trace), final_cycles // TICK_CC)
        for eid, ntok in sorted_remaining:
            print(eid, ntok)
        for mode, cand_id in trace:
            print(mode, cand_id)


if __name__ == "__main__":
    main()
