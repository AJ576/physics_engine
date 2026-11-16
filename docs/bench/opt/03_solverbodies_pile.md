A = build of the previous commit (bench/build_ref.sh)
B = build/release/physics_bench (this change)
scene=pile reps=9 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 7.937 | 7.809 | 0.992 | 0.937-1.038 | 7.720-8.249 | 7.547-8.182 | within noise |
