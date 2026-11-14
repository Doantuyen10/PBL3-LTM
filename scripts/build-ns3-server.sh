#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
NS3_DIR="$PROJECT_DIR/tools/ns-3.47"
SRC="$NS3_DIR/scratch/epuck-ns3-server.cc"
OUT_DIR="$NS3_DIR/build/scratch"
OUT_BIN="$OUT_DIR/ns3.47-epuck-ns3-server-default"

mkdir -p "$OUT_DIR"

g++ -std=c++23 -O2 -Wall -Wextra \
  -I"$NS3_DIR/build/include" \
  "$SRC" \
  -L"$NS3_DIR/build/lib" \
  -Wl,-rpath,"$NS3_DIR/build/lib" \
  -lns3.47-core-default \
  -lns3.47-network-default \
  -lns3.47-mobility-default \
  -lns3.47-propagation-default \
  -lns3.47-wifi-default \
  -lns3.47-spectrum-default \
  -lns3.47-antenna-default \
  -lns3.47-energy-default \
  -lns3.47-internet-default \
  -lns3.47-applications-default \
  -lns3.47-stats-default \
  -o "$OUT_BIN"

echo "[OK] Built ns-3 closed-loop server: $OUT_BIN"
