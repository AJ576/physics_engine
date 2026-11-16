A = build of the previous commit (bench/build_ref.sh) 
B = build/release/physics_bench 
scene=gas reps=7 steps=300, pinned to cpu 3, order alternates AB/BA
| N | A p50 median (ms) | B p50 median (ms) | median B/A | B/A range | A p50 range | B p50 range | verdict |
|---:|---:|---:|---:|---:|---:|---:|---|
| 5000 | 1.189 | 1.130 | 0.947 | 0.890-1.258 | 1.104-1.277 | 1.108-1.398 | within noise |
| 20000 | 5.475 | 5.534 | 1.001 | 0.963-1.122 | 5.244-5.546 | 5.267-5.884 | within noise |
