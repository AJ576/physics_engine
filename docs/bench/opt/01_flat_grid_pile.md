A = build/A/physics_bench (previous commit)
B = build/release/physics_bench (this change)
scene=pile reps=7 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 8.229 | 7.781 | 0.957 | 0.898-1.004 | 8.026-8.611 | 7.686-8.063 | within noise |
