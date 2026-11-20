#!/usr/bin/env python3
"""posture_summary.py - summarize a labeled cam_debug recording.

Analysis aid only (CLAUDE.md: Python is for tools/ support, not graded logic).
It reads the numbers that a recording already holds and computes per-posture
statistics. It does not detect anything and never touches frames or images.

Input: a text file made like this (see the recording steps in the chat/docs):

    ./build/cam_debug | while IFS= read -r l; do echo "$(date -u +%FT%T.%3NZ) $l"; done | tee -a FILE
    mark() { echo "$(date -u +%FT%T.%3NZ) LABEL $*" >> FILE; }

Lines it uses:
    <timestamp> LABEL <name>                     starts a hold (or "transition")
    <timestamp> ... | OK CVA=63.1 reject=none     a valid reading
    <timestamp> ... | NO_USER CVA=-1.0 reject=..  a second with no valid reading

A hold runs from its LABEL line to the next LABEL line (the "transition"
labels the timer script writes between holds end it), and at most --max-hold
seconds, so readings recorded after the timer finished are ignored.

For each hold it prints how many seconds had a valid reading and the CVA
statistics. "Settled" statistics skip the first --skip seconds of the hold,
because that is when the posture is still changing.

Usage:
    python3 tools/posture_summary.py ~/posture_run3.txt
    python3 tools/posture_summary.py --skip 10 run_a.txt run_b.txt
    python3 tools/posture_summary.py --selftest
"""
import argparse
import re
import statistics
import sys
from datetime import datetime, timedelta

TS_FMT = "%Y-%m-%dT%H:%M:%S.%fZ"
LINE_RE = re.compile(r"^(\S+)\s+(.*)$")
LABEL_RE = re.compile(r"^LABEL\s+(\S+)")
STATUS_RE = re.compile(r"\|\s*(OK|NO_USER)\s+CVA=(-?\d+(?:\.\d+)?)")
BOUNDARY_LABELS = {"transition"}


def parse_hold_data(lines, max_hold):
    """Returns a list of holds: {label, start, readings=[(secs_into_hold, cva or None)]}."""
    holds = []
    current = None
    for line in lines:
        m = LINE_RE.match(line.rstrip("\n"))
        if not m:
            continue
        try:
            t = datetime.strptime(m.group(1), TS_FMT)
        except ValueError:
            continue
        rest = m.group(2)
        lm = LABEL_RE.match(rest)
        if lm:
            if lm.group(1) in BOUNDARY_LABELS:
                current = None
            else:
                current = {"label": lm.group(1), "start": t, "readings": []}
                holds.append(current)
            continue
        sm = STATUS_RE.search(rest)
        if sm and current is not None:
            dt = (t - current["start"]).total_seconds()
            if 0 <= dt <= max_hold:
                cva = float(sm.group(2)) if sm.group(1) == "OK" else None
                current["readings"].append((dt, cva))
    return holds


def summarize_hold(hold, skip):
    readings = hold["readings"]
    valid = [c for _, c in readings if c is not None]
    settled = [c for dt, c in readings if c is not None and dt >= skip]
    return {
        "label": hold["label"],
        "start": hold["start"],
        "secs": len(readings),
        "valid": len(valid),
        "mean_all": statistics.mean(valid) if valid else None,
        "settled_n": len(settled),
        "settled_mean": statistics.mean(settled) if settled else None,
        "settled_min": min(settled) if settled else None,
        "settled_max": max(settled) if settled else None,
    }


def fmt(v, spec=".1f"):
    return "-" if v is None else format(v, spec)


def print_report(name, holds, skip):
    rows = [summarize_hold(h, skip) for h in holds]
    print(f"file: {name}")
    if not rows:
        print("  no labeled holds found (need LABEL lines other than 'transition')")
        return rows
    print(f"  settled = readings after the first {skip:g} s of each hold")
    print(f"  {'#':>2} {'start':>8}  {'label':<28} {'secs':>4} {'valid':>5} {'valid%':>6} "
          f"{'mean_all':>8} {'n':>3} {'settled':>8} {'range':>11}")
    for i, r in enumerate(rows, 1):
        pct = f"{100 * r['valid'] / r['secs']:.0f}%" if r["secs"] else "-"
        rng = "-" if r["settled_n"] == 0 else f"{r['settled_min']:.1f}-{r['settled_max']:.1f}"
        print(f"  {i:>2} {r['start'].strftime('%H:%M:%S'):>8}  {r['label']:<28} {r['secs']:>4} "
              f"{r['valid']:>5} {pct:>6} {fmt(r['mean_all']):>8} {r['settled_n']:>3} "
              f"{fmt(r['settled_mean']):>8} {rng:>11}")

    by_label = {}
    for r in rows:
        by_label.setdefault(r["label"], []).append(r)
    print("\n  by label (mean of the holds' settled means):")
    print(f"  {'label':<28} {'holds':>5} {'mean':>7} {'range of means':>15} {'valid%':>7}")
    for label, rs in by_label.items():
        means = [r["settled_mean"] for r in rs if r["settled_mean"] is not None]
        secs = sum(r["secs"] for r in rs)
        valid = sum(r["valid"] for r in rs)
        rng = "-" if not means else f"{min(means):.1f}-{max(means):.1f}"
        pct = f"{100 * valid / secs:.0f}%" if secs else "-"
        print(f"  {label:<28} {len(rs):>5} {fmt(statistics.mean(means) if means else None):>7} "
              f"{rng:>15} {pct:>7}")
    print()
    return rows


def selftest():
    """Synthetic recording with known answers (no real data involved)."""
    t0 = datetime(2026, 11, 20, 0, 0, 0)

    def ts(sec):
        return (t0 + timedelta(seconds=sec)).strftime(TS_FMT.replace("%fZ", "%f"))[:-3] + "Z"

    lines = ["%s cam_debug: header line that must be ignored" % ts(0)]
    lines.append("%s LABEL transition" % ts(1))
    lines.append("%s LABEL upright" % ts(2))
    for s in range(30):                       # hold 1: first 8 s at 60, then 70; every 10th second lost
        if s % 10 == 9:
            lines.append("%s purple=1 clusters=1 good=1 (9 px) | NO_USER CVA=-1.0 reject=not-two-clusters" % ts(2 + s + 0.5))
        else:
            cva = 60.0 if s < 8 else 70.0
            lines.append("%s purple=9 clusters=2 good=2 (9 px) | OK CVA=%.1f reject=none" % (ts(2 + s + 0.5), cva))
    lines.append("%s LABEL transition" % ts(40))
    lines.append("%s LABEL both" % ts(48))
    for s in range(10):                       # hold 2: ten seconds at 40
        lines.append("%s purple=9 clusters=2 good=2 (9 px) | OK CVA=40.0 reject=none" % ts(48 + s + 0.5))
    for s in range(100):                      # recorded after the timer ended: beyond max-hold, ignored
        lines.append("%s purple=9 clusters=2 good=2 (9 px) | OK CVA=1.0 reject=none" % ts(100 + s))

    holds = parse_hold_data(lines, max_hold=31.0)
    assert [h["label"] for h in holds] == ["upright", "both"], holds
    a = summarize_hold(holds[0], skip=8.0)
    assert a["secs"] == 30 and a["valid"] == 27, a                       # seconds 9, 19, 29 lost
    assert abs(a["mean_all"] - (8 * 60.0 + 19 * 70.0) / 27) < 1e-9, a   # 8 at 60 + 19 at 70
    assert a["settled_n"] == 19 and a["settled_mean"] == 70.0, a
    assert a["settled_min"] == 70.0 and a["settled_max"] == 70.0, a
    b = summarize_hold(holds[1], skip=8.0)
    assert b["secs"] == 10 and b["valid"] == 10 and b["settled_n"] == 2 and b["settled_mean"] == 40.0, b
    assert b["mean_all"] == 40.0, b
    print("posture_summary selftest: all checks passed")


def main(argv=None):
    p = argparse.ArgumentParser(description="Summarize a labeled cam_debug recording.")
    p.add_argument("files", nargs="*", help="recording file(s)")
    p.add_argument("--skip", type=float, default=8.0,
                   help="seconds at the start of each hold to leave out of 'settled' (default 8)")
    p.add_argument("--max-hold", type=float, default=31.0,
                   help="ignore readings later than this many seconds after a hold starts "
                        "(default 31: the timer holds each posture for 30 s)")
    p.add_argument("--selftest", action="store_true", help="run the built-in self-test and exit")
    args = p.parse_args(argv)

    if args.selftest:
        selftest()
        return 0
    if not args.files:
        p.print_usage(sys.stderr)
        print("posture_summary: give at least one recording file (or --selftest)", file=sys.stderr)
        return 2
    status = 0
    for name in args.files:
        try:
            with open(name, encoding="utf-8", errors="replace") as f:
                lines = f.readlines()
        except OSError as e:
            print(f"posture_summary: cannot read {name}: {e.strerror}", file=sys.stderr)
            status = 1
            continue
        print_report(name, parse_hold_data(lines, args.max_hold), args.skip)
    return status


if __name__ == "__main__":
    sys.exit(main())
