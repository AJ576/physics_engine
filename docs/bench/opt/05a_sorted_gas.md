A = build of the previous commit (bench/build_ref.sh)
B = build/release/physics_bench (this change)
scene=gas reps=7 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 1.148 | 1.190 | 1.058 | 0.911-1.174 | 1.098-1.335 | 1.162-1.349 | within noise |
| 20000 | 5.359 | 5.566 | 1.044 | 0.960-1.093 | 5.121-5.698 | 5.256-5.914 | within noise |
