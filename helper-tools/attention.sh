#!/bin/bash
# attention.sh — Ring terminal bell repeatedly until user presses Enter.
# Usage: ./attention.sh "Please lift the battery for hard power reset"
#
# The message (if provided) is printed once, then the bell rings every
# 2 seconds until the user hits Enter.

MSG="${1:-ATTENTION NEEDED — press Enter when ready}"

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  $MSG"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

while true; do
    printf '\a'  # BEL character (terminal bell)
    # Check if Enter was pressed (non-blocking read with 2s timeout)
    if read -t 2 -r; then
        echo ">> OK, continuing..."
        break
    fi
done
