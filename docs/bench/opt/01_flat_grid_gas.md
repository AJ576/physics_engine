A = build/A/physics_bench (previous commit)
B = build/release/physics_bench (this change)
scene=gas reps=9 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 1.714 | 1.145 | 0.666 | 0.583-0.910 | 1.685-1.893 | 1.103-1.549 | faster |
| 20000 | 9.623 | 5.303 | 0.545 | 0.535-0.576 | 9.441-9.935 | 5.208-5.635 | faster |
