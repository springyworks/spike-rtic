#!/usr/bin/env bash
# test_serial_linux.sh — Test host_synced fix with pure Linux commands
# Tests that the first command after opening serial is clean (no banner leak)
PORT=/dev/ttyACM0
PASS=0
FAIL=0

pass() { echo "  PASS: $1"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $1"; FAIL=$((FAIL+1)); }

# Configure port: raw mode, no echo, 1-second read timeout
stty -F "$PORT" 115200 raw -echo -echoe -echok -echoctl -echoke min 0 time 10

echo "=== Test 1: First 'info' command after fresh open ==="
# Open fd, send info, read response
exec 3<>"$PORT"
printf 'info\r\n' >&3
sleep 0.3
# Read up to a few lines with timeout
resp=""
while IFS= read -r -t 1 line <&3; do
    resp+="$line"$'\n'
done
exec 3>&-
echo "  Response: $(echo "$resp" | head -5)"
if echo "$resp" | grep -qi "rtic\|spike.*prime.*hub"; then
    if echo "$resp" | grep -qi "SPIKE Prime RTIC Monitor"; then
        fail "Banner leaked into info response"
    else
        pass "Clean info response, no banner"
    fi
else
    fail "No RTIC in response: $resp"
fi

sleep 0.5

echo "=== Test 2: 'version' after reconnect ==="
exec 3<>"$PORT"
printf 'version\r\n' >&3
sleep 0.3
resp=""
while IFS= read -r -t 1 line <&3; do
    resp+="$line"$'\n'
done
exec 3>&-
echo "  Response: $(echo "$resp" | head -3)"
if echo "$resp" | grep -q "^v"; then
    pass "Clean version response"
else
    fail "Unexpected version response: $resp"
fi

sleep 0.5

echo "=== Test 3: Bare LF (echo style) ==="
exec 3<>"$PORT"
# echo adds \n by default
echo "help" >&3
sleep 0.3
resp=""
while IFS= read -r -t 1 line <&3; do
    resp+="$line"$'\n'
done
exec 3>&-
echo "  Response: $(echo "$resp" | head -3)"
if echo "$resp" | grep -q -i "commands\|help\|info\|version\|motor"; then
    pass "LF-terminated command accepted"
else
    fail "LF command not recognized: $resp"
fi

sleep 0.5

echo "=== Test 4: Bare CR ==="
exec 3<>"$PORT"
printf 'info\r' >&3
sleep 0.3
resp=""
while IFS= read -r -t 1 line <&3; do
    resp+="$line"$'\n'
done
exec 3>&-
echo "  Response: $(echo "$resp" | head -3)"
if echo "$resp" | grep -qi "rtic\|spike.*prime.*hub"; then
    pass "CR-terminated command accepted"
else
    fail "CR command not recognized: $resp"
fi

sleep 0.5

echo "=== Test 5: Rapid reconnect ×3 ==="
all_clean=true
for i in 1 2 3; do
    exec 3<>"$PORT"
    printf 'info\r\n' >&3
    sleep 0.3
    resp=""
    while IFS= read -r -t 1 line <&3; do
        resp+="$line"$'\n'
    done
    exec 3>&-
    if echo "$resp" | grep -qi "SPIKE Prime RTIC Monitor"; then
        echo "  Iteration $i: BANNER LEAKED"
        all_clean=false
    else
        echo "  Iteration $i: clean"
    fi
    sleep 0.3
done
if $all_clean; then
    pass "All 3 rapid reconnects clean"
else
    fail "Banner leaked during rapid reconnects"
fi

sleep 0.5

echo "=== Test 6: printf to port file (no fd) ==="
# This is the simplest possible approach — write directly to device file
printf 'version\r\n' > "$PORT"
sleep 0.5
resp=""
exec 3<"$PORT"
while IFS= read -r -t 1 line <&3; do
    resp+="$line"$'\n'
done
exec 3>&-
echo "  Response: $(echo "$resp" | head -3)"
if echo "$resp" | grep -q "^v"; then
    pass "Direct printf > device works"
else
    # This one may not work reliably with CDC — don't count as hard fail
    echo "  SKIP: Direct write may not sync with CDC"
fi

echo ""
echo "=============================="
echo "Results: $PASS passed, $FAIL failed"
echo "=============================="
exit $FAIL
