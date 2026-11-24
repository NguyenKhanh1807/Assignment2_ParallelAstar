#!/usr/bin/env bash
# Step 1 on the Mac: download the whole Ho Chi Minh City road network with OSMnx.
# Usage (Terminal):  cd ~/Downloads/Assignment2_ParallelAstar && bash fetch_city_mac.sh
# Needs internet. Takes a few minutes. Result: data_city.zip next to this script.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
command -v python3 >/dev/null || { echo "python3 not found: run  xcode-select --install"; exit 1; }
if [ ! -d .venv_osm ]; then python3 -m venv .venv_osm; fi
. .venv_osm/bin/activate
pip install -q --upgrade pip
pip install -q "osmnx>=2.0,<3"
python3 fetch_city.py data_city
zip -q -r data_city.zip data_city
echo
echo "DONE -> $HERE/data_city.zip"
echo "Tell Claude it has finished."
