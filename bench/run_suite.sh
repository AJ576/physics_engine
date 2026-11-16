#!/usr/bin/env bash
# Full benchmark suite. Usage: bench/run_suite.sh OUTDIR [build_dir]
# Writes OUTDIR/env.txt, gas.txt/.csv (N sweep + brute force + 60 Hz search),
# pile.txt/.csv (N sweep with gravity) and noise.txt (10 repeated processes).
set -euo pipefail
OUT=$1; BUILD=${2:-build/release}; BIN=$BUILD/physics_bench
mkdir -p "$OUT"
PIN=""; command -v taskset >/dev/null && PIN="taskset -c $(( $(nproc) - 1 ))"
bench/env.sh "$BUILD" > "$OUT/env.txt"
echo "pinning:   ${PIN:-none}" >> "$OUT/env.txt"
$PIN $BIN --scene gas --brute --search --csv "$OUT/gas.csv" | tee "$OUT/gas.txt"
$PIN $BIN --scene pile --csv "$OUT/pile.csv" | tee "$OUT/pile.txt"
{
  echo "noise study: 10 separate processes, gas N=5000, 3 runs x 300 steps each"
  echo "columns: mean_ms p50_ms p99_ms"
  for i in $(seq 1 10); do $PIN $BIN --n 5000 --runs 3 --steps 300 | awk '$1==5000{print $2, $3, $4}'; done
} > "$OUT/noise.txt"
awk 'NR>2{for(i=1;i<=3;i++){s[i]+=$i; q[i]+=$i*$i; if(!(i in mn)||$i<mn[i])mn[i]=$i; if($i>mx[i])mx[i]=$i} n++}
     END{split("mean p50 p99",nm," "); for(i=1;i<=3;i++){m=s[i]/n; sd=sqrt((q[i]-n*m*m)/(n-1));
         printf "%s: avg %.4f ms, sd %.4f ms, CV %.1f%%, range %.4f-%.4f ms\n", nm[i], m, sd, 100*sd/m, mn[i], mx[i]}}' \
     "$OUT/noise.txt" | tee -a "$OUT/noise.txt"
