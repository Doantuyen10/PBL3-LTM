#!/usr/bin/env bash
set -euo pipefail

# Install locally in this project; no sudo or system-wide changes.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
version=3.47
install_dir="$project_dir/tools"
source_dir="$install_dir/ns-$version"
archive="$install_dir/ns-$version.tar.bz2"

for tool in curl tar python3 cmake g++; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "Missing required tool: $tool" >&2
    exit 1
  fi
done
if ! command -v ninja >/dev/null 2>&1 && ! command -v make >/dev/null 2>&1; then
  echo 'Install ninja or make before continuing.' >&2
  exit 1
fi
mkdir -p "$install_dir"
if [[ ! -d "$source_dir" ]]; then
  if [[ ! -f "$archive" ]]; then
    echo "Downloading ns-$version from nsnam.org..."
    curl --fail --location --retry 2 --connect-timeout 20 \
      "https://www.nsnam.org/releases/ns-$version.tar.bz2" -o "$archive.part"
    mv -- "$archive.part" "$archive"
  fi
  tar -tjf "$archive" >/dev/null
  tar -xjf "$archive" -C "$install_dir"
fi
cd -- "$source_dir"
./ns3 configure --enable-examples --enable-tests --disable-python-bindings
./ns3 build -j "${NS3_JOBS:-2}"
./ns3 run hello-simulator
./ns3 run first
python3 test.py --suite=core
printf '\nns-3 ready: %s\n' "$source_dir"
