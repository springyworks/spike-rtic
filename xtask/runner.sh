#!/bin/bash
# Runner for `cargo run` — delegates to the xtask hub manager.
#
# Cargo invokes this with the built ARM ELF path as $1.
# We forward to `cargo xtask run-elf` which shows an interactive
# flash/upload dialog instead of trying to execute the binary on x86.

set -euo pipefail

ELF="$1"
shift

# Ensure PROJECT_ROOT is set
if [ -z "${PROJECT_ROOT:-}" ]; then
    DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    export PROJECT_ROOT="$DIR"
fi

# Fast path: run pre-compiled xtask binary directly (avoids cargo overhead)
XTASK_BIN="$PROJECT_ROOT/xtask/target/x86_64-unknown-linux-gnu/debug/xtask"
if [ -x "$XTASK_BIN" ]; then
    exec "$XTASK_BIN" run-elf "$ELF"
fi

# Slow path: build & run via cargo (first invocation only)
echo "  Building xtask (first time)..."
exec cargo run \
    --manifest-path "$PROJECT_ROOT/xtask/Cargo.toml" \
    --target x86_64-unknown-linux-gnu \
    --quiet -- run-elf "$ELF"
