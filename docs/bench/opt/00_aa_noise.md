A = identical copy of build/release/physics_bench
B = build/release/physics_bench (this change)
scene=gas reps=9 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 1.750 | 1.754 | 1.011 | 0.940-1.145 | 1.670-1.892 | 1.695-1.942 | within noise |
| 20000 | 9.666 | 9.803 | 1.016 | 0.958-1.067 | 9.266-10.063 | 9.590-10.415 | within noise |
