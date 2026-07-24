#!/usr/bin/env python3
"""Generate exact continuation-score vectors from the current policy model."""

from __future__ import annotations

import random
import sys
from pathlib import Path


TICK_CC = 11_264
N_TESTS = 2_048

IDEA_MODEL = Path(__file__).resolve().parents[2] / "Idea_Model"
sys.path.insert(0, str(IDEA_MODEL))

import scheduler_hw_fixed_policy as golden  # noqa: E402


def exact_ticks(cycles: int) -> int:
    assert cycles % TICK_CC == 0
    return cycles // TICK_CC


def ceil_ticks(cycles: int) -> int:
    # The cycle-domain golden may split divisible tail work at half a Tq.
    # RTL timestamps are integer Tq ticks, so the frozen hardware contract
    # rounds the final continuation score upward.
    return (cycles + TICK_CC - 1) // TICK_CC


def make_remaining(rng: random.Random, tid: int) -> tuple[tuple[int, int], ...]:
    # The first 65 vectors force every possible remaining length.  Later
    # vectors concentrate additional coverage on the exact-tail boundaries.
    if tid <= 64:
        rem_len = tid
    else:
        rem_len = rng.choice((0, 1, 2, 3, 4, 5, 8, 16, 32, 64, rng.randrange(65)))
    ntoks = sorted((rng.randrange(1, 257) for _ in range(rem_len)), reverse=True)
    return tuple((eid, ntok) for eid, ntok in enumerate(ntoks))


def main() -> None:
    rng = random.Random(0x5C0E_2026)
    print(N_TESTS)
    for tid in range(N_TESTS):
        remaining = make_remaining(rng, tid)
        c2_ticks = rng.randrange(0, 20_001)
        c3_ticks = rng.randrange(0, 20_001)
        c2 = golden.cm._cc_idle_at(c2_ticks * TICK_CC)
        c3 = golden.cm._cc_idle_at(c3_ticks * TICK_CC)

        total_conc = sum(exact_ticks(golden.cm._cc_best_conc(ntok)) for _, ntok in remaining)
        total_task = sum(exact_ticks(golden.cm._cc_best_task(ntok)) for _, ntok in remaining)
        max_conc = exact_ticks(golden.cm._cc_best_conc(remaining[0][1])) if remaining else 0
        rem0 = remaining[0][1] if remaining else 0
        rem1 = remaining[1][1] if len(remaining) > 1 else 0
        expected_cycles = golden.hw_v2_continuation(
            c2, c3, remaining, policy="balanced"
        )
        expected = ceil_ticks(expected_cycles)

        fields = [
            tid,
            c2_ticks,
            c3_ticks,
            len(remaining),
            rem0,
            rem1,
            total_conc,
            max_conc,
            total_task,
        ]
        for slot in range(4):
            if slot < len(remaining):
                fields.extend((1, remaining[slot][1]))
            else:
                fields.extend((0, 0))
        fields.append(expected)
        print(" ".join(str(value) for value in fields))


if __name__ == "__main__":
    main()
