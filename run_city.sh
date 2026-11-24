#!/usr/bin/env bash
# Parts 1 and 3 on the whole-city graph (data_city/). Part 2 does not use the graph and is
# taken from run_all.sh. Usage: ./run_city.sh <platform-name>
set -e
cd "$(dirname "$0")"
P=${1:-$(uname -s)-$(uname -m)-$(getconf _NPROCESSORS_ONLN)c}
N=$(getconf _NPROCESSORS_ONLN)
T="1"; t=2; while [ $t -le $((2*N)) ] && [ $t -le 16 ]; do T="$T,$t"; t=$((t*2)); done
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
