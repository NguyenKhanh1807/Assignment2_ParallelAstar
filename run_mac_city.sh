#!/usr/bin/env bash
# Native macOS run of Parts 1 and 3 on the whole-city graph.
# Usage (Terminal):  cd ~/Downloads/Assignment2_ParallelAstar && bash run_mac_city.sh
# Takes about 10-15 minutes. Please do not run other heavy programs meanwhile.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
xcrun --find clang++ >/dev/null 2>&1 || { echo "clang++ not found: run  xcode-select --install"; exit 1; }
rm -rf mac_city 2>/dev/null || true
mkdir -p mac_city && tar xzf a2city_src.tgz -C mac_city
cd mac_city/a2city
CHIP=$(sysctl -n machdep.cpu.brand_string | tr ' ' '-' | tr 'A-Z' 'a-z')
P="macos-native-${CHIP}-$(sysctl -n hw.ncpu)c"
caffeinate -i -w $$ 2>/dev/null &
CXX=clang++ make -s all
CXX=clang++ ./run_city.sh "$P"
cd results_city && zip -q "$HERE/results_city_$P.zip" *"_$P".*
echo
echo "DONE -> $HERE/results_city_$P.zip"
echo "Tell Claude it has finished."
