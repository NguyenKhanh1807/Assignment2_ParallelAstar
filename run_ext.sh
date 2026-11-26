#!/usr/bin/env bash
# Extended benchmarks addressing methodology & coverage gaps:
#  1) bench_clist: RANGE sweep (1000, 10000, 50000) + HeapPQ baseline + variance stats (--reps 5) + pinning
#  2) bench_astar: pool vs spawn, physical-core pinning, TTASLock, TIME metric, hda-window sweep,
#                  per-query p50/p95/p99, batch IQR/std, retired bytes & RSS (--reps 7)
set -e
cd "$(dirname "$0")"
P=${1:-$(uname -s)-$(uname -m)-$(getconf _NPROCESSORS_ONLN)c}
N=$(getconf _NPROCESSORS_ONLN)
T="1"; t=2; h_added=0; f_added=0
T_NONOVER="1"
while [ $t -le $((2*N)) ] && [ $t -le 32 ]; do
  if [ "$N" -gt 8 ]; then
    if [ $h_added -eq 0 ] && [ $((N/2)) -lt $t ] && [ $((N/2)) -gt 4 ]; then
      T="$T,$((N/2))"; [ $((N/2)) -le "$N" ] && T_NONOVER="$T_NONOVER,$((N/2))"; h_added=1
    fi
    if [ $f_added -eq 0 ] && [ "$N" -lt $t ] && [ "$N" -gt 4 ]; then
      T="$T,$N"; T_NONOVER="$T_NONOVER,$N"; f_added=1
    fi
  fi
  T="$T,$t"
  [ $t -le "$N" ] && T_NONOVER="$T_NONOVER,$t"
  [ $((N/2)) -eq $t ] && h_added=1; [ "$N" -eq $t ] && f_added=1
  t=$((t*2))
done
if [ "$N" -gt 8 ] && [ $f_added -eq 0 ] && [ "$N" -le 32 ]; then T="$T,$N"; T_NONOVER="$T_NONOVER,$N"; fi

echo "extended run platform=$P cores=$N threads=$T (non-oversubscribed=$T_NONOVER)"
mkdir -p results results_city
make -s all

# 1. Part 2 extended: RANGE sweep + HeapPQ + stats (--no-ttas when oversubscribed/large range)
CL_EXT="results/clist_ext_${P}.csv"
rm -f "$CL_EXT"
./bin/bench_clist --platform "$P" --threads "$T_NONOVER" --range 1000 --reps 5 --pin --include-heap --stats-out "$CL_EXT" >/dev/null
if [ "$T" != "$T_NONOVER" ]; then
  T_OVER=$(echo "$T" | tr ',' '\n' | awk -v n="$N" '$1>n' | paste -sd,)
  [ -n "$T_OVER" ] && ./bin/bench_clist --platform "$P" --threads "$T_OVER" --range 1000 --reps 5 --pin --include-heap --no-ttas --stats-out "$CL_EXT" --no-header >/dev/null
fi
for R in 10000 50000; do
  ./bin/bench_clist --platform "$P" --threads "$T" --range "$R" --reps 5 --pin --include-heap --no-ttas --stats-out "$CL_EXT" --no-header >/dev/null
done

# 2. Part 3 extended: stats, pool vs spawn, pinning, TTAS, TIME metric, window sweep
AST_EXT="results_city/astar_ext_${P}.csv"
rm -f "$AST_EXT"

COMMON="--platform $P --data data_city --threads $T --min-dist 5000 --queries 20 --reps 7"
PAR_VERS="list-lazy-backoff,heap-backoff,hda,hda-window"

# 2a. Spawn (unpinned) vs Pool (unpinned) vs Pool+Pinned on core contenders (LENGTH)
for H in euclid alt8 euclid-costly; do
  NH=""
  [ -s "$AST_EXT" ] && NH="--no-header"
  ./bin/bench_astar $COMMON --heuristic "$H" --vers "seq-binary-heap,$PAR_VERS" --stats-out "$AST_EXT" $NH >/dev/null
  ./bin/bench_astar $COMMON --heuristic "$H" --vers "$PAR_VERS" --pool --stats-out "$AST_EXT" --no-header >/dev/null
  ./bin/bench_astar $COMMON --heuristic "$H" --vers "$PAR_VERS" --pool --pin --stats-out "$AST_EXT" --no-header >/dev/null
done

# 2b. TTASLock coverage (non-oversubscribed threads, skip fine-ttas to avoid lock-chain hang)
for H in euclid euclid-costly; do
  ./bin/bench_astar --platform "$P" --data data_city --threads "$T_NONOVER" --min-dist 5000 --queries 20 --reps 7 \
    --heuristic "$H" --include-ttas --skip-fine-ttas --only-ver ttas --pool --pin --stats-out "$AST_EXT" --no-header >/dev/null
done

# 2c. TIME metric coverage (pool + pin)
for H in euclid alt8 euclid-costly; do
  ./bin/bench_astar $COMMON --metric time --heuristic "$H" --vers "seq-binary-heap,$PAR_VERS" --pool --pin --stats-out "$AST_EXT" --no-header >/dev/null
done

# 2d. HDA* f-window sweep (0.00, 0.01, 0.03, 0.10)
for H in euclid euclid-costly; do
  ./bin/bench_astar $COMMON --heuristic "$H" --windows 0.0,0.01,0.03,0.10 --only-ver hda-w --pool --pin --stats-out "$AST_EXT" --no-header >/dev/null
done

echo "extended done -> $CL_EXT , $AST_EXT"
