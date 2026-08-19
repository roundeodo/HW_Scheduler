#!/usr/bin/env python3
"""Measure exact RTL scheduler-round/lowering overlap from a Questa run."""

import argparse
import json
import re
from collections import Counter, deque


TRACE_RE = re.compile(
    r"\[MOE_SCHED_RTL_TRACE\]\s+time_ns=(?P<time>[0-9.]+)\s+"
    r"cycle=(?P<cycle>[0-9]+)\s+scope=(?P<scope>\w+)\s+"
    r"event=(?P<event>\w+)(?P<fields>.*)$"
)
FIELD_RE = re.compile(r"(\w+)=([^\s]+)")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--transcript", required=True)
    parser.add_argument("--bingo-trace", required=True)
    return parser.parse_args()


def parse_rtl_events(path):
    events = []
    with open(path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = TRACE_RE.search(line)
            if not match:
                continue
            fields = dict(FIELD_RE.findall(match.group("fields")))
            events.append(
                {
                    "time": float(match.group("time")),
                    "cycle": int(match.group("cycle")),
                    "scope": match.group("scope"),
                    "event": match.group("event"),
                    "fields": fields,
                }
            )
    return events


def parse_lowering(path):
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)
    duration_events = [
        event
        for event in data["traceEvents"]
        if event.get("ph") == "X" and event.get("tid") == "Host Core"
    ]
    sched = [
        event
        for event in duration_events
        if event.get("name") == "BINGO_TRACE_HOST_MOE_HW_SCHED"
    ]
    if len(sched) != 1:
        raise ValueError("expected one HOST_MOE_HW_SCHED event, got %d" % len(sched))
    sched_start = float(sched[0]["ts"])
    sched_end = sched_start + float(sched[0]["dur"])
    lower = []
    for event in duration_events:
        if event.get("name") != "BINGO_TRACE_HOST_MOE_HW_LOWER":
            continue
        start = float(event["ts"])
        end = start + float(event["dur"])
        if start < sched_start or end > sched_end:
            continue
        lower.append(
            {
                "start": start,
                "end": end,
                "dur": end - start,
                "host_cycles": int(event.get("args", {}).get("dur_cc", 0)),
            }
        )
    return sorted(lower, key=lambda item: item["start"]), (sched_start, sched_end)


def pair_intervals(events, scope, start_event, done_event):
    pending = deque()
    intervals = []
    for event in events:
        if event["scope"] != scope:
            continue
        if event["event"] == start_event:
            pending.append(event)
        elif event["event"] == done_event:
            if not pending:
                raise ValueError("unmatched %s/%s" % (scope, done_event))
            start = pending.popleft()
            intervals.append(
                {
                    "start": start["time"],
                    "end": event["time"],
                    "dur": event["time"] - start["time"],
                    "cycles": event["cycle"] - start["cycle"],
                    "start_event": start,
                    "done_event": event,
                }
            )
    if pending:
        raise ValueError("unmatched %s/%s" % (scope, start_event))
    return intervals


def overlap(a_start, a_end, b_start, b_end):
    return max(0.0, min(a_end, b_end) - max(a_start, b_start))


def total_overlap(lhs, rhs):
    total = 0.0
    for left in lhs:
        for right in rhs:
            total += overlap(left["start"], left["end"], right["start"], right["end"])
    return total


def interval_overlap(item, intervals):
    return sum(
        overlap(item["start"], item["end"], other["start"], other["end"])
        for other in intervals
    )


def pct(part, whole):
    return 0.0 if whole == 0 else 100.0 * part / whole


def cycles(ns, period_ns):
    return ns / period_ns


def main():
    args = parse_args()
    rtl = parse_rtl_events(args.transcript)
    lower, software_sched = parse_lowering(args.bingo_trace)
    rounds = pair_intervals(rtl, "core", "ROUND_START", "ROUND_DONE")
    evals = pair_intervals(rtl, "round", "EVAL_START", "EVAL_DONE")
    bounds = pair_intervals(rtl, "round", "BOUND_START", "BOUND_DONE")
    compares = pair_intervals(rtl, "round", "COMPARE_START", "COMPARE_DONE")
    s4_checks = pair_intervals(rtl, "round", "S4_START", "S4_DONE")

    if not rounds:
        raise ValueError("no RTL rounds found")
    if not lower:
        raise ValueError("no lowering events found")

    period_samples = [item["dur"] / item["cycles"] for item in rounds if item["cycles"]]
    period_ns = sum(period_samples) / len(period_samples)
    round_work_ns = sum(item["dur"] for item in rounds)
    lower_work_ns = sum(item["dur"] for item in lower)
    exact_overlap_ns = total_overlap(rounds, lower)
    production_span_ns = rounds[-1]["end"] - rounds[0]["start"]
    inter_round_ns = production_span_ns - round_work_ns
    first_lower_lead_ns = lower[0]["start"] - rounds[0]["start"]
    lowering_tail_ns = lower[-1]["end"] - rounds[-1]["end"]

    event_counts = Counter((event["scope"], event["event"]) for event in rtl)
    fifo_values = []
    for event in rtl:
        if event["scope"] != "core" or event["event"] not in ("TASK_PUSH", "TASK_POP"):
            continue
        if "fifo_after" in event["fields"]:
            fifo_values.append(int(event["fields"]["fifo_after"]))

    lower_classes = Counter()
    for item in lower:
        item_overlap = interval_overlap(item, rounds)
        if item_overlap == 0:
            lower_classes["none"] += 1
        elif abs(item_overlap - item["dur"]) < 0.001:
            lower_classes["full"] += 1
        else:
            lower_classes["partial"] += 1

    print("MoE HW Scheduler RTL vs CVA6 Lowering Overlap")
    print("=" * 72)
    print("RTL events              : %d" % len(rtl))
    print("Scheduler rounds        : %d" % len(rounds))
    print("Lowering tasks          : %d" % len(lower))
    print("Scheduler clock period  : %.3f ns" % period_ns)
    print("Software HW_SCHED span  : %.0f ns (%.1f cycles)" % (
        software_sched[1] - software_sched[0],
        cycles(software_sched[1] - software_sched[0], period_ns),
    ))
    print()
    print("Exact work comparison")
    print("-" * 72)
    print("Round-engine active work: %.0f ns = %.1f scheduler cycles" % (
        round_work_ns, cycles(round_work_ns, period_ns)))
    print("Lowering body work      : %.0f ns = %.1f host cycles" % (
        lower_work_ns, sum(item["host_cycles"] for item in lower)))
    print("Lowering / round work   : %.3fx" % (lower_work_ns / round_work_ns))
    print("Exact simultaneous work : %.0f ns = %.1f cycles" % (
        exact_overlap_ns, cycles(exact_overlap_ns, period_ns)))
    print("Lowering overlapped     : %.2f%%" % pct(exact_overlap_ns, lower_work_ns))
    print("Round work overlapped   : %.2f%%" % pct(exact_overlap_ns, round_work_ns))
    print()
    print("Pipeline behavior")
    print("-" * 72)
    print("First-to-last round span: %.0f ns = %.1f cycles" % (
        production_span_ns, cycles(production_span_ns, period_ns)))
    print("Inter-round/non-eval gap: %.0f ns = %.1f cycles" % (
        inter_round_ns, cycles(inter_round_ns, period_ns)))
    print("Scheduler head start    : %.0f ns = %.1f cycles before first lowering" % (
        first_lower_lead_ns, cycles(first_lower_lead_ns, period_ns)))
    print("Lowering final tail     : %.0f ns = %.1f cycles after final round" % (
        lowering_tail_ns, cycles(lowering_tail_ns, period_ns)))
    print("Lowering overlap classes: full=%d partial=%d none=%d" % (
        lower_classes["full"], lower_classes["partial"], lower_classes["none"]))
    print("Maximum observed FIFO   : %d / 8" % (max(fifo_values) if fifo_values else -1))
    print("Task pushes / pops      : %d / %d" % (
        event_counts[("core", "TASK_PUSH")], event_counts[("core", "TASK_POP")]))
    print()
    print("Submodule work")
    print("-" * 72)
    for label, intervals in (
        ("transition evaluation", evals),
        ("bound score", bounds),
        ("pair compare", compares),
        ("target S4 search", s4_checks),
    ):
        duration = sum(item["dur"] for item in intervals)
        print("%-23s count=%4d work=%9.0f ns (%8.1f cycles) lower_overlap=%6.2f%%" % (
            label,
            len(intervals),
            duration,
            cycles(duration, period_ns),
            pct(total_overlap(intervals, lower), duration),
        ))

    print()
    print("Per-round timeline")
    print("-" * 104)
    print("idx  remaining remove  start_ns   end_ns     cycles  lower_overlap_cc overlap_pct")
    for index, item in enumerate(rounds, 1):
        fields_start = item["start_event"]["fields"]
        fields_done = item["done_event"]["fields"]
        item_overlap = interval_overlap(item, lower)
        print("%3d  %9s %6s  %9.0f  %9.0f  %7d  %16.1f %10.2f" % (
            index,
            fields_start.get("remaining", "?"),
            fields_done.get("remove", "?"),
            item["start"],
            item["end"],
            item["cycles"],
            cycles(item_overlap, period_ns),
            pct(item_overlap, item["dur"]),
        ))

    print()
    print("Per-lowering timeline")
    print("-" * 104)
    print("idx  start_ns   end_ns     host_cc  round_overlap_cc overlap_pct active_rounds")
    for index, item in enumerate(lower, 1):
        item_overlap = interval_overlap(item, rounds)
        active_rounds = [
            str(round_index)
            for round_index, round_item in enumerate(rounds, 1)
            if overlap(item["start"], item["end"], round_item["start"], round_item["end"])
        ]
        print("%3d  %9.0f  %9.0f  %7d  %16.1f %10.2f %s" % (
            index,
            item["start"],
            item["end"],
            item["host_cycles"],
            cycles(item_overlap, period_ns),
            pct(item_overlap, item["dur"]),
            ",".join(active_rounds) if active_rounds else "-",
        ))


if __name__ == "__main__":
    main()
