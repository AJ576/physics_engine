A = build of the previous commit (bench/build_ref.sh)
B = build/release/physics_bench (this change)
scene=pile reps=9 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 7.861 | 7.284 | 0.946 | 0.824-1.044 | 7.468-8.686 | 7.074-8.638 | within noise |
