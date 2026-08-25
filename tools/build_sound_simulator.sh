#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

CJSON_DIR="${CJSON_DIR:-}"
if [[ -z "$CJSON_DIR" ]]; then
  CJSON_DIR=$(find "${HOME}/.platformio/packages" -type d -path "*/components/json/cJSON" 2>/dev/null | head -1)
fi
if [[ -z "$CJSON_DIR" || ! -f "$CJSON_DIR/cJSON.c" ]]; then
  echo "cJSON not found. Set CJSON_DIR or install PlatformIO ESP-IDF package." >&2
  exit 1
fi

g++ -std=c++17 -g -D LINUX \
  -I../include -I../lib/SUMD/src -I../lib/RcEcuBus/src -I../lib/sigslot/src -I"$CJSON_DIR" \
  sound_simulator.cpp "$CJSON_DIR/cJSON.c" \
  -o sound_simulator \
  -lportaudio -lncurses
