A = build of the previous commit (bench/build_ref.sh)
B = build/release/physics_bench (this change)
scene=gas reps=7 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 1.171 | 1.178 | 1.003 | 0.755-1.265 | 1.116-1.683 | 1.116-1.499 | within noise |
| 20000 | 5.454 | 5.478 | 0.996 | 0.912-1.062 | 5.183-6.006 | 5.283-5.713 | within noise |
