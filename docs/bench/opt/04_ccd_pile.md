A = build of the previous commit (bench/build_ref.sh)
B = build/release/physics_bench (this change)
scene=pile reps=7 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 7.999 | 8.079 | 1.013 | 0.950-1.048 | 7.671-8.392 | 7.794-8.468 | within noise |
