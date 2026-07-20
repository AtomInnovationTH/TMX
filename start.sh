#!/usr/bin/env bash
set -euo pipefail

# Resolve paths relative to this script so it works from any CWD.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FQBN="arduino:samd:mzero_bl"
HEX="$SCRIPT_DIR/build/chat/lora-chat.ino.hex"

# Optional: pass a specific port to avoid flashing the wrong board, e.g.
#   ./start.sh /dev/cu.usbmodem14201
PORT="${1:-}"

# Make sure arduino-cli exists before doing anything else.
if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "arduino-cli not found in PATH. Install it first: https://arduino.github.io/arduino-cli/" >&2
  exit 1
fi

if [ ! -f "$HEX" ]; then
  echo "Hex file not found: $HEX (build it first)" >&2
  exit 1
fi

# 1. find the port (unless one was given explicitly)
if [ -z "$PORT" ]; then
  shopt -s nullglob
  PORTS=(/dev/cu.usbmodem*)
  shopt -u nullglob

  if [ "${#PORTS[@]}" -eq 0 ]; then
    echo "No board found (no /dev/cu.usbmodem* port). Plug it in and try again." >&2
    exit 1
  fi

  if [ "${#PORTS[@]}" -gt 1 ]; then
    echo "Multiple boards found. Re-run with the one you want:" >&2
    printf '  ./start.sh %s\n' "${PORTS[@]}" >&2
    exit 1
  fi

  PORT="${PORTS[0]}"
elif [ ! -e "$PORT" ]; then
  echo "Given port does not exist: $PORT" >&2
  exit 1
fi

# 2. flash (Board 2 is a GENERIC board -> build/chat)
# Note: mzero_bl does a 1200bps touch-reset; arduino-cli re-detects the
# bootloader port automatically, so passing the application port is correct.
echo "Flashing $HEX to $PORT ..."
arduino-cli upload -b "$FQBN" -p "$PORT" --input-file "$HEX"
echo "Done."
