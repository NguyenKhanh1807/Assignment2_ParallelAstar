#!/usr/bin/env bash
# Native macOS run (third shared-memory platform).
# Usage (in Terminal):  cd ~/Downloads/Assignment2_ParallelAstar && bash run_mac.sh
# Needs the Xcode Command Line Tools (clang, make). Takes about 5-10 minutes.
# Result: results_<platform>.zip next to this script.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

if ! xcrun --find clang++ >/dev/null 2>&1; then
  echo "clang++ not found. Install the Command Line Tools first:  xcode-select --install"; exit 1
fi

# Fresh copy of the project, so earlier experiments in this folder are not disturbed.
WORK="$HERE/mac_run"
if [ ! -f "$WORK/Assignment2_ParallelAstar/Makefile" ]; then
  mkdir -p "$WORK"
  ditto -x -k "$HERE/Assignment2_ParallelAstar.zip" "$WORK"
fi
cd "$WORK/Assignment2_ParallelAstar"

CHIP=$(sysctl -n machdep.cpu.brand_string | tr ' ' '-' | tr 'A-Z' 'a-z')   # e.g. apple-m2-pro
NCPU=$(sysctl -n hw.ncpu)
P="macos-native-${CHIP}-${NCPU}c"
echo "== platform: $P"

caffeinate -i -w $$ 2>/dev/null &   # keep the Mac awake while this script runs
make -s clean >/dev/null 2>&1 || true
CXX=clang++ make -s all
CXX=clang++ ./run_all.sh "$P"

{ echo "platform=$P (native macOS, Apple clang)"
  sw_vers; uname -a; clang++ --version | head -1
  sysctl -n machdep.cpu.brand_string
  echo "hw.ncpu=$(sysctl -n hw.ncpu)"
  echo "performance cores=$(sysctl -n hw.perflevel0.physicalcpu 2>/dev/null)"
  echo "efficiency cores=$(sysctl -n hw.perflevel1.physicalcpu 2>/dev/null)"
  echo "memory_bytes=$(sysctl -n hw.memsize)"
} > "results/platform_$P.txt"

cd results
zip -q "$HERE/results_$P.zip" *"_$P".*
echo
echo "DONE -> $HERE/results_$P.zip"
echo "Tell Claude it has finished."
