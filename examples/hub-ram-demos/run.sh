#!/usr/bin/env bash
#
# Cargo runner for SPIKE Prime hub RAM demos.
#
# Called by cargo as:  run.sh <path-to-elf> [args...]
#
# Flow: objcopy ELF→bin → upload via COBS → send "go" → stream output
#
set -euo pipefail

ELF="$1"
shift || true

# Resolve paths relative to this script (hub-ram-demos/)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
UPLOAD_PY="$SCRIPT_DIR/../../helper-tools/upload_demo.py"

if [ ! -f "$ELF" ]; then
    echo "ERROR: ELF not found: $ELF" >&2
    exit 1
fi

# Create .bin next to the ELF
BIN="${ELF}.bin"

echo "=== SPIKE Runner ==="
echo "  ELF:  $ELF"

# objcopy to raw binary
if command -v rust-objcopy &>/dev/null; then
    rust-objcopy -O binary "$ELF" "$BIN"
elif command -v arm-none-eabi-objcopy &>/dev/null; then
    arm-none-eabi-objcopy -O binary "$ELF" "$BIN"
else
    echo "ERROR: need rust-objcopy or arm-none-eabi-objcopy" >&2
    exit 1
fi

SIZE=$(stat -c%s "$BIN" 2>/dev/null || stat -f%z "$BIN" 2>/dev/null)
echo "  BIN:  $BIN ($SIZE bytes)"

# Upload and run
if [ ! -f "$UPLOAD_PY" ]; then
    echo "ERROR: upload_demo.py not found at $UPLOAD_PY" >&2
    echo "  (copy finished .bin manually and use the shell)" >&2
    exit 1
fi

WAIT="${SPIKE_RUN_TIMEOUT:-30}"
echo "  Uploading + running (timeout ${WAIT}s)..."
echo ""
exec python3 "$UPLOAD_PY" "$BIN" "$WAIT"
