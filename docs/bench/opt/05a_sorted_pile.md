A = build of the previous commit (bench/build_ref.sh)
B = build/release/physics_bench (this change)
scene=pile reps=9 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 8.158 | 9.007 | 1.111 | 1.064-1.200 | 7.849-8.691 | 8.894-9.781 | slower |
