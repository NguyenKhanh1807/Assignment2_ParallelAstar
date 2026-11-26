#!/usr/bin/env bash
# Parts 1 and 3 on the whole-city graph (data_city/). Part 2 does not use the graph and is
# taken from run_all.sh. Usage: ./run_city.sh <platform-name>
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
DATA=data_city; OUT=results_city
echo "platform=$P cores=$N threads=$T data=$DATA out=$OUT"
mkdir -p "$OUT"
make -s all
./bin/test_seq "$DATA"        | tee "$OUT"/test_seq_$P.txt | tail -1
./bin/test_par "$DATA" 8 30   | tee "$OUT"/test_par_$P.txt | tail -1
./bin/bench_seq --platform "$P" --data "$DATA" --out "$OUT"
echo "platform,version,heuristic,threads,total_ms,avg_expanded,avg_ratio,max_ratio" > "$OUT"/astar_$P.csv
for H in euclid alt8 euclid-costly zero; do
  ./bin/bench_astar --platform "$P" --threads "$T" --data "$DATA" --min-dist 5000 --queries 20 --reps 3 \
      --heuristic "$H" --quick --no-header >> "$OUT"/astar_$P.csv
done
{ echo "platform=$P"; uname -a; (lscpu 2>/dev/null || sysctl -n machdep.cpu.brand_string hw.ncpu hw.memsize 2>/dev/null); } > "$OUT"/platform_$P.txt
echo "done -> $OUT/*_$P.*"
