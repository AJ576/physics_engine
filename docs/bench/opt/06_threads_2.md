A = build/release/physics_bench --threads 1
B = build/release/physics_bench --threads 2
scene=gas reps=7 steps=300, pinned to cpu 0-3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 1.208 | 1.246 | 1.031 | 0.787-1.287 | 1.119-1.583 | 1.235-1.509 | within noise |
| 20000 | 5.680 | 5.176 | 0.895 | 0.833-1.083 | 5.515-6.021 | 5.014-5.973 | within noise |
| 50000 | 16.126 | 14.743 | 0.930 | 0.897-1.038 | 15.200-16.398 | 14.468-15.771 | within noise |
