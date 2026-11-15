A = build of the previous commit (bench/build_ref.sh)
B = build/release/physics_bench (this change)
scene=gas reps=9 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 1.165 | 1.159 | 1.039 | 0.880-1.049 | 1.106-1.311 | 1.133-1.273 | within noise |
| 20000 | 5.343 | 5.380 | 1.002 | 0.939-1.237 | 5.193-5.579 | 5.215-6.744 | within noise |
