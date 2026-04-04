#!/usr/bin/env python3
"""
Color Ballet — Host-side choreography script for motor + color sensor.

Drives Motor B to rotate a colored object past the LEGO color sensor
on port F.  The script searches for red and green (or any two target
colors), then elegantly alternates between them with slow, smooth
motions — all controlled through the hub's interactive shell commands.

Unlike RAM demos (which run on the hub), this script runs on the host
and sends shell commands (`motor <port> <speed>`, `sensor f`) over
serial.  This demonstrates the shell's real-time control capabilities
from Python without any binary upload.

Color sensor mode 0 returns discrete color IDs (LEGO LPF2):
  0=black, 1=magenta(?), 2=?, 3=blue, 4=?, 5=green,
  6=?, 7=yellow, 8=?, 9=red, 10=white, -1=none

Contents:
  - open_hub()    — open serial, flush banner, return serial handle
  - cmd()         — send shell command, return response text
  - motor_set()   — set Motor B speed via shell
  - read_color()  — read color sensor ID from port F via shell
  - scan_for()    — rotate Motor B until target color found
  - main loop     — alternate between two colors with smooth transitions

We use mode 5 (RGB_I) for finer control, but mode 0 for the initial
color-name search since it gives clean IDs.
"""

import serial
import time
import re
import sys

PORT = "/dev/ttyACM0"

# Color IDs from LEGO color sensor mode 0
COLOR_NONE = -1
COLOR_BLACK = 0
COLOR_RED = 9
COLOR_GREEN = 5
COLOR_YELLOW = 7
COLOR_WHITE = 10
COLOR_BLUE = 3

COLOR_NAMES = {
    -1: "none", 0: "black", 1: "magenta?", 2: "?2", 3: "blue",
    4: "?4", 5: "green", 6: "?6", 7: "yellow", 8: "?8",
    9: "red", 10: "white",
}

# Which two colors to alternate between (use what's on the turntable)
TARGET_A = COLOR_RED
TARGET_B = COLOR_YELLOW


def open_hub():
    s = serial.Serial(PORT, 115200, timeout=2)
    time.sleep(0.5)
    s.reset_input_buffer()
    s.write(b"\r\n")
    time.sleep(0.3)
    s.read(s.in_waiting or 4096)  # flush
    return s


def cmd(s, text, wait=0.5):
    """Send a command, return response text."""
    s.reset_input_buffer()
    s.write((text + "\r\n").encode())
    time.sleep(wait)
    return s.read(s.in_waiting or 4096).decode("ascii", errors="replace")


def motor(s, port, speed):
    """Set motor speed (-100..100) or 'brake'."""
    if isinstance(speed, str):
        cmd(s, f"motor {port} {speed}", 0.2)
    else:
        cmd(s, f"motor {port} {speed}", 0.2)


def motor_brake(s, port):
    cmd(s, f"motor {port} brake", 0.2)


def read_color_id(s):
    """Read color sensor in mode 0, return integer color ID."""
    resp = cmd(s, "sensor", 0.3)
    # Look for "value=<number>"
    m = re.search(r"value=(-?\d+)", resp)
    if m:
        return int(m.group(1))
    return None


def read_rgb(s):
    """Read color sensor in mode 5, return (R, G, B, I) tuple."""
    resp = cmd(s, "sensor", 0.3)
    m = re.search(r"R=(\d+)\s+G=(\d+)\s+B=(\d+)\s+I=(\d+)", resp)
    if m:
        return tuple(int(x) for x in m.groups())
    return None


def probe_sensor(s):
    """Probe color sensor on port F, wait for sync."""
    print("Probing color sensor on port F...")
    cmd(s, "sensor f", 1.0)
    # Wait for sensor to sync — can take 3-5 seconds
    for i in range(20):
        time.sleep(0.5)
        resp = cmd(s, "sensor", 0.5)
        if "value=" in resp or "R=" in resp:
            print(f"  Sensor online: {resp.strip()}")
            return True
        if "Syncing" in resp:
            print(f"  ({i}: syncing...)")
    print("  WARNING: sensor may not be synced yet")
    return False


def set_mode(s, mode):
    """Switch sensor mode."""
    cmd(s, f"sensor mode {mode}", 0.5)
    time.sleep(0.3)


# ── Smooth speed ramping ────────────────────────────────────

def ramp_speed(s, port, start, end, duration=1.0, steps=20):
    """Smoothly ramp motor speed from start to end over duration seconds."""
    dt = duration / steps
    for i in range(steps + 1):
        t = i / steps
        # Ease-in-out (sine curve)
        import math
        ease = (1 - math.cos(t * math.pi)) / 2
        spd = int(start + (end - start) * ease)
        motor(s, port, spd)
        time.sleep(dt)


# ── Search phase ────────────────────────────────────────────

def search_colors(s):
    """
    Slowly rotate motor D, scanning for red and green objects.
    Returns dict with color positions (approximate tick timestamps).
    """
    print("\n--- Phase 1: Searching for colors ---")
    set_mode(s, 0)  # discrete color mode
    time.sleep(0.5)

    found = {}  # {color_id: first_seen_time}
    scan_speed = 12  # slow search speed

    # Ensure we're in color ID mode
    set_mode(s, 0)
    time.sleep(0.5)
    # Verify mode switch took effect
    for _ in range(5):
        resp = cmd(s, "sensor", 0.3)
        if "value=" in resp:
            break
        time.sleep(0.3)

    print(f"  Rotating at speed {scan_speed}...")
    motor(s, "b", scan_speed)

    start = time.time()
    timeout = 30.0  # search for up to 30 seconds
    last_color = None

    while time.time() - start < timeout:
        cid = read_color_id(s)
        if cid is not None and cid != COLOR_NONE and cid != COLOR_BLACK:
            name = COLOR_NAMES.get(cid, f"?{cid}")
            if cid != last_color:
                elapsed = time.time() - start
                print(f"  [{elapsed:5.1f}s] Detected: {name} (id={cid})")
                last_color = cid
                if cid in (TARGET_A, TARGET_B) and cid not in found:
                    found[cid] = elapsed

        # Stop once we found both targets
        if TARGET_A in found and TARGET_B in found:
            print(f"  Found both {COLOR_NAMES[TARGET_A]} and {COLOR_NAMES[TARGET_B]}!")
            break

        time.sleep(0.03)
    else:
        motor_brake(s, "b")
        if not found:
            print("  No target colors found in scan.")
            return None
        missing = []
        if TARGET_A not in found:
            missing.append(COLOR_NAMES[TARGET_A])
        if TARGET_B not in found:
            missing.append(COLOR_NAMES[TARGET_B])
        if missing:
            print(f"  WARNING: did not find: {', '.join(missing)}")

    motor_brake(s, "b")
    time.sleep(0.3)
    return found


# ── Seek to color ───────────────────────────────────────────

def seek_color(s, target_id, speed=8, timeout=15.0):
    """
    Slowly rotate until we see the target color.
    Returns True if found.
    """
    name = COLOR_NAMES.get(target_id, f"?{target_id}")
    motor(s, "b", speed)
    start = time.time()
    while time.time() - start < timeout:
        cid = read_color_id(s)
        if cid == target_id:
            motor_brake(s, "b")
            time.sleep(0.1)
            return True
        time.sleep(0.02)
    motor_brake(s, "b")
    return False


def creep_to_center(s, target_id, speed=8):
    """
    Nudge the motor very slowly to center the target color
    under the sensor. Checks color, micro-steps until stable.
    """
    # Already on color — just verify
    cid = read_color_id(s)
    if cid == target_id:
        return True

    # Tiny nudges
    for _ in range(30):
        motor(s, "b", speed)
        time.sleep(0.04)
        motor_brake(s, "b")
        time.sleep(0.06)
        cid = read_color_id(s)
        if cid == target_id:
            return True
    return False


# ── The Ballet ──────────────────────────────────────────────

def ballet(s, cycles=6):
    """
    Alternate between red and green with elegant, slow motions.
    Each transition: ramp up slowly, coast until color appears,
    ramp down, pause to admire.
    """
    print("\n--- Phase 2: The Ballet ---")
    set_mode(s, 0)  # discrete color mode
    time.sleep(0.3)

    targets = [TARGET_A, TARGET_B]
    direction = 1  # +1 or -1

    for cycle in range(cycles):
        target = targets[cycle % 2]
        name = COLOR_NAMES[target]

        print(f"\n  Cycle {cycle+1}/{cycles}: seeking {name}...")

        # Gentle ramp up
        top_speed = 10 * direction
        ramp_speed(s, "b", 0, top_speed, duration=1.0, steps=15)

        # Coast until we find the target
        found = False
        start = time.time()
        while time.time() - start < 15.0:
            cid = read_color_id(s)
            if cid == target:
                found = True
                break
            time.sleep(0.02)

        if found:
            # Gentle deceleration
            current_speed = top_speed
            ramp_speed(s, "b", current_speed, 0, duration=0.6, steps=12)
            motor_brake(s, "b")
            time.sleep(0.1)

            # Fine-center on the color
            creep_to_center(s, target, speed=6 * direction)

            # Read and display RGB values for beauty
            set_mode(s, 5)
            time.sleep(0.4)
            rgb = read_rgb(s)
            set_mode(s, 0)
            time.sleep(0.3)

            if rgb:
                r, g, b, i = rgb
                print(f"  Arrived at {name}: R={r} G={g} B={b} I={i}")
            else:
                print(f"  Arrived at {name}")

            # Pause to admire — longer for elegance
            pause = 1.5 + 0.3 * cycle  # gradually longer pauses
            print(f"  Holding for {pause:.1f}s...")
            time.sleep(pause)
        else:
            print(f"  Could not find {name}, reversing...")
            ramp_speed(s, "b", top_speed, 0, duration=0.5, steps=10)
            motor_brake(s, "b")
            time.sleep(0.5)

        # Alternate direction each cycle for elegance
        direction *= -1

    # Final rest
    motor_brake(s, "b")
    print("\n  Ballet complete.")


# ── Main ────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  Color Ballet — Red & Green Alternation")
    print("  Motor B + Color Sensor F")
    print("=" * 60)

    s = open_hub()

    resp = cmd(s, "uptime")
    print(f"Hub: {resp.strip()}")

    # Probe color sensor
    if not probe_sensor(s):
        print("Continuing anyway...")

    # Search phase
    found = search_colors(s)
    if found is None:
        print("\nAborting — no colors found.")
        motor_brake(s, "b")
        s.close()
        return

    # Ballet phase
    ballet(s, cycles=8)

    # Clean up
    motor_brake(s, "b")
    cmd(s, "sensor stop", 0.5)

    resp = cmd(s, "uptime")
    print(f"\nHub uptime: {resp.strip()}")
    print("Done.")
    s.close()


if __name__ == "__main__":
    main()
