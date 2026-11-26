#!/usr/bin/env bash
# Build, test and benchmark everything on the current machine (one "platform").
# Usage: ./run_all.sh <platform-name>      e.g. ./run_all.sh macbook-m2-8c
# Optional environment: DATA=<graph dir> (default data), OUT=<results dir> (default results),
#                       MINDIST=<min straight-line distance of Part 3 queries in m> (default 3000)
set -e
cd "$(dirname "$0")"
P=${1:-$(uname -s)-$(uname -m)-$(getconf _NPROCESSORS_ONLN)c}
N=$(getconf _NPROCESSORS_ONLN)
T="1"; t=2; h_added=0; f_added=0
while [ $t -le $((2*N)) ] && [ $t -le 32 ]; do
  if [ "$N" -gt 8 ]; then
    if [ $h_added -eq 0 ] && [ $((N/2)) -lt $t ] && [ $((N/2)) -gt 4 ]; then T="$T,$((N/2))"; h_added=1; fi
    if [ $f_added -eq 0 ] && [ "$N" -lt $t ] && [ "$N" -gt 4 ]; then T="$T,$N"; f_added=1; fi
  fi
  T="$T,$t"
  [ $((N/2)) -eq $t ] && h_added=1; [ "$N" -eq $t ] && f_added=1
  t=$((t*2))
done
if [ "$N" -gt 8 ] && [ $f_added -eq 0 ] && [ "$N" -le 32 ]; then T="$T,$N"; fi
DATA=${DATA:-data}; OUT=${OUT:-results}; MINDIST=${MINDIST:-3000}
echo "platform=$P cores=$N threads=$T data=$DATA out=$OUT"
mkdir -p "$OUT"
make -s all
./bin/test_seq "$DATA"            | tee "$OUT"/test_seq_$P.txt   | tail -1
./bin/test_clist 8 --no-ttas   | tee "$OUT"/test_clist_$P.txt | tail -1
./bin/test_par "$DATA" 8 60       | tee "$OUT"/test_par_$P.txt   | tail -1
./bin/bench_seq --platform "$P" --data "$DATA" --out "$OUT"
# benchmarks are run in chunks (one list family / one heuristic at a time)
echo "platform,list,threads,mix,range,mops" > "$OUT"/clist_$P.csv
for L in coarse fine optimistic lazy; do
  ./bin/bench_clist --platform "$P" --threads "$T" --only "$L-" --no-header >> "$OUT"/clist_$P.csv
done
echo "platform,version,heuristic,threads,total_ms,avg_expanded,avg_ratio,max_ratio" > "$OUT"/astar_$P.csv
for H in zero euclid alt8 euclid-costly; do
  ./bin/bench_astar --platform "$P" --threads "$T" --data "$DATA" --min-dist "$MINDIST" --queries 40 --reps 3 --heuristic "$H" --no-header >> "$OUT"/astar_$P.csv
done
{ echo "platform=$P"; uname -a; (lscpu 2>/dev/null || sysctl -n machdep.cpu.brand_string hw.ncpu hw.memsize 2>/dev/null); } > "$OUT"/platform_$P.txt
echo "done -> "$OUT"/*_$P.*"
