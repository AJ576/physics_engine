A = build/release/physics_bench --threads 1
B = build/release/physics_bench --threads 4
scene=gas reps=7 steps=300, pinned to cpu 0-3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 1.215 | 1.236 | 1.009 | 0.930-1.211 | 1.120-1.308 | 1.135-1.410 | within noise |
| 20000 | 5.792 | 5.092 | 0.885 | 0.833-0.952 | 5.432-6.617 | 4.882-5.512 | faster |
| 50000 | 15.928 | 14.756 | 0.936 | 0.870-0.971 | 15.332-16.967 | 13.610-15.427 | faster |
