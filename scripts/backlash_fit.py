#!/usr/bin/env python3
"""
backlash_fit.py

Fits the bidirectional reversal deadband (delta_x, delta_y) and corner
overshoot from an optical-encoder capture taken while jogging each axis back
and forth across the same point.

Method: pair each reversal with the samples immediately before and after it.
The commanded position is continuous through the reversal, the encoder is not.
The gap between encoder-before and encoder-after, minus the commanded travel,
is the deadband. We take the median over all reversals to reject the odd
sample where the scale skipped a count.

Usage:
    python scripts/backlash_fit.py data/reversal_encoder_log.csv
"""

import argparse
import csv
import statistics
import sys
from collections import defaultdict

MIN_REVERSAL_TRAVEL_MM = 0.5   # ignore reversals with almost no motion
OVERSHOOT_SETTLE_MM = 0.005    # corner is "settled" inside this band


def load(path):
    per_axis = defaultdict(list)
    with open(path, newline="", encoding="utf-8") as fh:
        for rec in csv.DictReader(fh):
            per_axis[rec["axis"].strip().upper()].append(
                {
                    "cmd": float(rec["commanded_mm"]),
                    "enc": float(rec["encoder_mm"]),
                    "feed": float(rec["feedrate_mm_min"]),
                }
            )
    return per_axis


def find_reversals(samples):
    """
    Yield indices i where the commanded direction flips between i-1 and i.

    A repeated point in the log (dwell row) makes the delta zero, so we carry
    the last non-zero direction forward instead of resetting on it.
    """
    out = []
    last_dir = 0.0
    for i in range(1, len(samples)):
        d = samples[i]["cmd"] - samples[i - 1]["cmd"]
        if d == 0.0:
            continue
        direction = 1.0 if d > 0.0 else -1.0
        if last_dir != 0.0 and direction != last_dir:
            if abs(d) >= MIN_REVERSAL_TRAVEL_MM:
                out.append(i)
        last_dir = direction
    return out


def deadband(samples, reversals):
    """
    For each reversal the true carriage position must be continuous, so:
        deadband = |enc_after - enc_before| - |cmd_after - cmd_before|
    reported as a positive number when the encoder lags.
    """
    values = []
    for i in reversals:
        enc_jump = samples[i]["enc"] - samples[i - 1]["enc"]
        cmd_jump = samples[i]["cmd"] - samples[i - 1]["cmd"]
        residual = abs(enc_jump) - abs(cmd_jump)
        values.append(-residual)   # lag shows up as negative residual
    if not values:
        return 0.0, 0.0, 0
    return statistics.median(values), max(values), len(values)


def corner_overshoot(samples, reversals):
    """
    Peak tracking error in the window after each reversal, i.e. how far the
    encoder deviates from the commanded line before it settles back inside
    OVERSHOOT_SETTLE_MM. With a lashy nut this is dominated by the deadband.
    """
    worst = 0.0
    for i in reversals:
        for j in range(i, min(i + 40, len(samples))):
            err = abs(samples[j]["enc"] - samples[j]["cmd"])
            if err < OVERSHOOT_SETTLE_MM:
                break
            worst = max(worst, err)
    return worst


def feed_sensitivity(samples, reversals):
    """Deadband difference between slow and fast segments (lo - hi)."""
    if not reversals:
        return 0.0
    lo = [s for s in samples if s["feed"] < 600.0]
    hi = [s for s in samples if s["feed"] >= 600.0]
    lo_rev = find_reversals(lo)
    hi_rev = find_reversals(hi)
    if not lo_rev or not hi_rev:
        return 0.0
    lo_db, _, _ = deadband(lo, lo_rev)
    hi_db, _, _ = deadband(hi, hi_rev)
    return lo_db - hi_db


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("capture", help="encoder reversal CSV log")
    ap.add_argument("--json", action="store_true", help="machine readable output")
    args = ap.parse_args(argv)

    per_axis = load(args.capture)
    if not per_axis:
        print("no rows parsed", file=sys.stderr)
        return 1

    results = {}
    for axis in sorted(per_axis):
        samples = per_axis[axis]
        revs = find_reversals(samples)
        med, peak, n = deadband(samples, revs)
        over = corner_overshoot(samples, revs)
        results[axis] = {
            "reversals": n,
            "deadband_median_mm": round(med, 5),
            "deadband_peak_mm": round(peak, 5),
            "corner_overshoot_mm": round(over, 5),
            "feed_sensitivity_mm": round(feed_sensitivity(samples, revs), 5),
        }

    if args.json:
        import json

        print(json.dumps(results, indent=2))
        return 0

    print(f"{'axis':<6}{'revs':>6}{'median':>10}{'peak':>10}{'overshoot':>12}{'feed_sens':>11}")
    for axis, r in results.items():
        print(
            f"{axis:<6}{r['reversals']:>6}"
            f"{r['deadband_median_mm']:>10.4f}"
            f"{r['deadband_peak_mm']:>10.4f}"
            f"{r['corner_overshoot_mm']:>12.4f}"
            f"{r['feed_sensitivity_mm']:>11.4f}"
        )

    print()
    print("Compensation to apply in the controller:")
    for axis, r in results.items():
        half = r["deadband_median_mm"] / 2.0
        print(f"  {axis}: BACKLASH = {half:.4f} mm per side")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
