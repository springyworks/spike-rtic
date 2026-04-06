#!/usr/bin/env python3
"""Send 'gdb' command to SPIKE hub to enter RSP mode, then release the port.

Usage:  python3 enter_rsp.py [/dev/ttyACM0]

This script:
  1. Opens the serial port
  2. Sends 'gdb\r\n' to enter GDB RSP mode
  3. Waits for the '+' ACK (RSP ready)
  4. Closes the port so GDB can open it

Run this BEFORE launching GDB / VS Code debug session.
"""

import serial
import serial.tools.list_ports
import sys
import time

LEGO_VID    = 0x0694
RUNTIME_PID = 0x0042

def find_hub_port():
    """Auto-detect LEGO SPIKE hub serial port by VID:PID."""
    for p in serial.tools.list_ports.comports():
        if (p.vid or 0) == LEGO_VID and (p.pid or 0) == RUNTIME_PID:
            return p.device
    return None

PORT = sys.argv[1] if len(sys.argv) > 1 else find_hub_port()
if not PORT:
    print("LEGO SPIKE hub not found (VID:PID 0694:0042). Is it plugged in?", file=sys.stderr)
    sys.exit(1)

try:
    ser = serial.Serial(PORT, timeout=2)
    # Flush any pending data
    ser.reset_input_buffer()

    # Send gdb command to enter RSP mode
    ser.write(b"gdb\r\n")
    ser.flush()

    # Wait briefly for the hub to switch modes
    time.sleep(0.3)

    # Read any response (should see RSP ready indicator)
    resp = ser.read(100)
    print(f"Hub response: {resp!r}")

    ser.close()
    print(f"RSP mode entered on {PORT} — port released for GDB.")

except serial.SerialException as e:
    print(f"Serial error: {e}", file=sys.stderr)
    print("Is the hub connected? Is another program using the port?", file=sys.stderr)
    sys.exit(1)
except KeyboardInterrupt:
    sys.exit(0)
