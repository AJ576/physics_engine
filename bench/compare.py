#!/usr/bin/env python3
"""Paired, interleaved A/B benchmark of two physics_bench binaries.

For each N and each of `reps` rounds, run A and B back to back (order
alternates every round: AB, BA, AB, ...), each as a separate process with
one run of `steps` timed steps after the usual warm-up. Each round gives a
paired ratio B/A of the p50 step time. Report the median ratio, its range,
and a verdict: "faster" only if every paired ratio is below 1, "slower" only
if every ratio is above 1, otherwise "within noise".

Usage: bench/compare.py A_BIN B_BIN [--scene gas|pile] [--n 5000,20000]
                        [--reps 9] [--steps 300] [--cpu 3] [--a-extra ARGS] [--b-extra ARGS]
"""
import argparse
import statistics
import subprocess
import sys


def run(binary, scene, n, steps, cpu, extra=""):
    cmd = [binary, "--scene", scene, "--n", str(n), "--runs", "1", "--steps", str(steps)] + extra.split()
    if cpu is not None:
        cmd = ["taskset", "-c", str(cpu)] + cmd
    out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    for line in out.splitlines():
        f = line.split()
        if f and f[0] == str(n):
            return float(f[1]), float(f[2])  # mean, p50
    raise RuntimeError("no result line for N=%d in:\n%s" % (n, out))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--scene", default="gas")
    ap.add_argument("--n", default="5000,20000")
    ap.add_argument("--reps", type=int, default=9)
    ap.add_argument("--steps", type=int, default=300)
    ap.add_argument("--cpu", default="3", help="taskset CPU list, e.g. 3 or 0-3")
    ap.add_argument("--a-extra", default="", help="extra physics_bench arguments for A, e.g. '--threads 1'")
    ap.add_argument("--b-extra", default="", help="extra physics_bench arguments for B")
    args = ap.parse_args()

    print("A = %s %s\nB = %s %s\nscene=%s reps=%d steps=%d, pinned to cpu %s, order alternates AB/BA"
          % (args.a, args.a_extra, args.b, args.b_extra, args.scene, args.reps, args.steps, args.cpu))
    print("| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |")
    print("|---:|---:|---:|---:|---:|---:|---:|---|")
    for n in [int(x) for x in args.n.split(",")]:
        pa, pb, ratios = [], [], []
        for r in range(args.reps):
            order = [("a", args.a, args.a_extra), ("b", args.b, args.b_extra)]
            if r % 2:
                order.reverse()
            res = {}
            for tag, binary, extra in order:
                res[tag] = run(binary, args.scene, n, args.steps, args.cpu, extra)[1]
            pa.append(res["a"])
            pb.append(res["b"])
            ratios.append(res["b"] / res["a"])
        verdict = "faster" if max(ratios) < 1 else ("slower" if min(ratios) > 1 else "within noise")
        print("| %d | %.3f | %.3f | %.3f | %.3f-%.3f | %.3f-%.3f | %.3f-%.3f | %s |"
              % (n, statistics.median(pa), statistics.median(pb), statistics.median(ratios), min(ratios),
                 max(ratios), min(pa), max(pa), min(pb), max(pb), verdict))
        sys.stdout.flush()


if __name__ == "__main__":
    main()
