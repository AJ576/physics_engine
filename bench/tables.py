#!/usr/bin/env python3
"""Print markdown tables from physics_bench CSV files (no hand-copied numbers).

Usage: bench/tables.py BASELINE_DIR FINAL_DIR
Each directory holds gas.csv and pile.csv written by bench/run_suite.sh.
"""
import csv
import os
import sys


def load(path):
    rows, extra = [], {}
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                parts = line[1:].strip().split(",")
                extra[parts[0]] = parts[1:]
    with open(path) as f:
        for r in csv.DictReader(l for l in f if not l.startswith("#")):
            rows.append(r)
    return {int(r["n"]): r for r in rows}, extra


def ms(x):
    return "%.3f" % float(x)


def main():
    base_dir, final_dir = sys.argv[1], sys.argv[2]
    bg, bx = load(os.path.join(base_dir, "gas.csv"))
    fg, fx = load(os.path.join(final_dir, "gas.csv"))
    bp, _ = load(os.path.join(base_dir, "pile.csv"))
    fp, _ = load(os.path.join(final_dir, "pile.csv"))

    print("### Gas scene: step time vs N (ms; median over 7 runs of 300 steps)\n")
    print("| N | old engine p50 | old p99 | new engine p50 | new mean | new p99 | new max | new run-to-run sd |")
    print("|---:|---:|---:|---:|---:|---:|---:|---:|")
    for n in sorted(fg):
        b, f = bg.get(n), fg[n]
        print("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            "{:,}".format(n), ms(b["p50_ms"]) if b else "-", ms(b["p99_ms"]) if b else "-",
            ms(f["p50_ms"]), ms(f["mean_ms"]), ms(f["p99_ms"]), ms(f["max_ms"]), ms(f["mean_sd_ms"])))

    print("\n### Grid vs brute force (gas scene)\n")
    print("| N | brute-force pairs | old grid candidates | new grid candidates | old: brute / grid time | new: brute / grid time |")
    print("|---:|---:|---:|---:|---:|---:|")
    for n in sorted(fg):
        b, f = bg.get(n), fg[n]
        bs = "%.1fx (%s / %s ms)" % (float(b["brute_mean_ms"]) / float(b["mean_ms"]), ms(b["brute_mean_ms"]), ms(b["mean_ms"])) if b else "-"
        fs = "%.1fx (%s / %s ms)" % (float(f["brute_mean_ms"]) / float(f["mean_ms"]), ms(f["brute_mean_ms"]), ms(f["mean_ms"]))
        print("| %s | %s | %s | %s | %s | %s |" % (
            "{:,}".format(n), "{:,}".format(int(float(f["brute_pairs"]))),
            "{:,}".format(int(float(b["cand_pairs"]))) if b else "-", "{:,}".format(int(float(f["cand_pairs"]))), bs, fs))

    print("\n### Largest N with the physics step inside 16.667 ms (60 Hz), gas scene\n")
    print("| criterion | old engine | new engine |")
    print("|---|---:|---:|")
    for key, label in (("largest_n_p99_under_16.667ms", "p99 step time"), ("largest_n_mean_under_16.667ms", "mean step time")):
        b, f = bx.get(key), fx.get(key)
        print("| %s | %s (next tested %s failed) | %s (next tested %s failed) |" % (
            label, "{:,}".format(int(b[0])) if b else "-", "{:,}".format(int(b[1])) if b else "-",
            "{:,}".format(int(f[0])) if f else "-", "{:,}".format(int(f[1])) if f else "-"))

    print("\n### Pile scene (gravity, e = 0.7): step time vs N (ms)\n")
    print("| N | old engine p50 | old p99 | new engine p50 | new p99 | new contacts per step |")
    print("|---:|---:|---:|---:|---:|---:|")
    for n in sorted(fp):
        b, f = bp.get(n), fp[n]
        print("| %s | %s | %s | %s | %s | %s |" % (
            "{:,}".format(n), ms(b["p50_ms"]) if b else "-", ms(b["p99_ms"]) if b else "-", ms(f["p50_ms"]),
            ms(f["p99_ms"]), "{:,}".format(int(float(f["contacts"])))))


if __name__ == "__main__":
    main()
