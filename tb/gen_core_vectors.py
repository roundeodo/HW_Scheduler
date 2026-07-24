#!/usr/bin/env python3
"""Generate per-round winner-token traces from the current policy model."""

from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path


TICK_CC = 11_264
N_TESTS = 512

IDEA_MODEL = Path(__file__).resolve().parents[2] / "Idea_Model"
sys.path.insert(0, str(IDEA_MODEL))

import scheduler_rtl_prefetch_both_policy as model  # noqa: E402


DEFAULT_COVERAGE_INPUTS = tuple(
    IDEA_MODEL / f"scheduler_strategy_coverage_E{experts}.json"
    for experts in (8, 32, 64)
)


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


def emit_case(
    tid: int,
    token_dist: dict[int, int],
    cache2: int,
    cache3: int,
) -> None:
    state = model.initial_state(token_dist, cache2, cache3)
    sorted_remaining = state.remaining
    trace: list[tuple[int, int]] = []
    while state.remaining:
        chosen = model.choose_transition(state)
        trace.append(model.token_from_tag(state, chosen.tag))
        state = chosen.state

    final_cycles = model.terminal_cost(state)
    assert final_cycles % TICK_CC == 0
    print(tid, len(sorted_remaining), cache2, cache3, len(trace), final_cycles // TICK_CC)
    for eid, ntok in sorted_remaining:
        print(eid, ntok)
    for mode, cand_id in trace:
        print(mode, cand_id)


def load_coverage_cases(paths: tuple[Path, ...]) -> list[tuple[dict[int, int], int, int]]:
    cases = []
    for path in paths:
        for case in json.loads(path.read_text())["cases"]:
            if not case.get("analysis_eligible", False):
                continue
            cases.append(
                (
                    {int(eid): int(ntok) for eid, ntok in case["dist"].items()},
                    int(case.get("c2", -1)),
                    int(case.get("c3", -1)),
                )
            )
    return cases


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--coverage-30k",
        action="store_true",
        help="emit all 29,928 analysis-eligible E8/E32/E64 coverage cases",
    )
    parser.add_argument("--coverage-input", action="append", type=Path)
    args = parser.parse_args()

    if args.coverage_30k:
        paths = tuple(args.coverage_input) if args.coverage_input else DEFAULT_COVERAGE_INPUTS
        cases = load_coverage_cases(paths)
        if len(cases) != 29_928:
            raise RuntimeError(f"expected 29928 eligible cases, got {len(cases)}")
        print(len(cases))
        for tid, (token_dist, cache2, cache3) in enumerate(cases):
            emit_case(tid, token_dist, cache2, cache3)
        return

    rng = random.Random(0xC0DE_6006)
    print(N_TESTS)
    for tid in range(N_TESTS):
        token_dist, cache2, cache3 = make_case(rng, tid)
        emit_case(tid, token_dist, cache2, cache3)


if __name__ == "__main__":
    main()
