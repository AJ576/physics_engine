A = build of the previous commit (bench/build_ref.sh)
B = build/release/physics_bench (this change)
scene=gas reps=7 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 1.150 | 1.141 | 0.989 | 0.935-1.190 | 1.102-1.252 | 1.129-1.326 | within noise |
| 20000 | 5.316 | 5.477 | 1.047 | 0.972-1.088 | 5.103-5.526 | 5.372-5.786 | within noise |
