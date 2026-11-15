#!/usr/bin/env bash
# Build physics_bench from a given commit into OUTDIR (for bench/compare.py).
# Usage: bench/build_ref.sh COMMIT OUTDIR
set -euo pipefail
COMMIT=$1; OUT=$(realpath -m "$2")
WT=$(mktemp -d)
git worktree add -q --detach "$WT" "$COMMIT"
cmake -S "$WT" -B "$WT/build" -DCMAKE_BUILD_TYPE=Release -DPHYS_BUILD_DEMO=OFF >/dev/null
cmake --build "$WT/build" --target physics_bench -j >/dev/null
mkdir -p "$OUT" && cp "$WT/build/physics_bench" "$OUT/physics_bench"
git worktree remove --force "$WT"
echo "$OUT/physics_bench ($(git rev-parse --short "$COMMIT"))"
