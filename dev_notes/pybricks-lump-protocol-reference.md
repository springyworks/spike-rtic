# Pybricks LUMP Protocol — Sensor Commands Reference

Extracted from Pybricks source (2026-04-02) for SPIKE RTIC firmware development.
Source: `pybricks-micropython/lib/pbio/src/port_lump.c`, `lib/lego/lego/lump.h`, `lib/lego/lego/device.h`

---

## SPIKE Sensor Type IDs

| Sensor                | Type ID (dec) | Type ID (hex) |
|-----------------------|:------------:|:-------------:|
| SPIKE Color Sensor    | 61           | 0x3D          |
| SPIKE Ultrasonic      | 62           | 0x3E          |
| SPIKE Force Sensor    | 63           | 0x3F          |
| SPIKE M Motor         | 48           | 0x30          |
| SPIKE L Motor         | 49           | 0x31          |
| SPIKE S Motor         | 65           | 0x41          |

---

## Header Byte Encoding

Every non-SYS LUMP message starts with a header byte:

```
HEADER = (MSG_TYPE & 0xC0) | (MSG_SIZE & 0x38) | (CMD_or_MODE & 0x07)
```

| Field     | Bits  | Values |
|-----------|-------|--------|
| MSG_TYPE  | [7:6] | SYS=0x00, CMD=0x40, INFO=0x80, DATA=0xC0 |
| MSG_SIZE  | [5:3] | 1B=0x00, 2B=0x08, 4B=0x10, 8B=0x18, 16B=0x20, 32B=0x28 |
| CMD/MODE  | [2:0] | Command type (for CMD) or mode index (for DATA/INFO) |

Size decode: `payload_bytes = 1 << ((SIZE_BITS >> 3) & 0x7)`

Total message = 1 (header) + payload_bytes + 1 (checksum)
For INFO messages: + 1 extra info-type byte after header.

---

## Checksum

```
checksum = 0xFF
for each byte in [header, payload_bytes...]:
    checksum ^= byte
// message = [header, payload..., checksum]
```

SYS messages (SYNC, NACK, ACK) are 1 byte only, **no checksum**.

---

## System Messages (no checksum, single byte)

| Message | Byte | Direction | Purpose |
|---------|:----:|-----------|---------|
| SYNC    | 0x00 | sensor→host | Sensor starts handshake |
| NACK    | 0x02 | host→sensor | **Keepalive** — must send every ≤100ms |
| ACK     | 0x04 | both | Acknowledge handshake complete |

---

## Handshake Flow (from port_lump.c sync thread)

1. **Sensor sends SYNC** (0x00) at 2400 baud
2. **Host optionally sends SPEED** command to test 115200 baud (for Powered Up)
3. **Sensor sends TYPE** — `[0x40, type_id, checksum]`
4. **Sensor sends CMD_MODES** — number of modes
5. **Sensor sends CMD_SPEED** — desired baud rate (115200 for LPF2)
6. **Sensor sends CMD_VERSION** — fw/hw version
7. **Sensor sends INFO** messages — for each mode: name, format, ranges, etc.
8. **Host sends ACK** (0x04) — signals end of handshake
9. **Baud rate changes** to the negotiated speed (115200)
10. **Host sends initial mode SELECT** — sets the default operating mode
11. **Data stream begins** — sensor sends DATA messages, host sends NACK keepalive

**Initial baud: 2400.** After ACK, both sides switch to 115200.

---

## Mode Select Command

Sent by host to switch the sensor's operating mode.

For modes 0–7:
```
[HEADER, mode, checksum]

HEADER = CMD(0x40) | SIZE_1(0x00) | SELECT(0x03) = 0x43
checksum = 0xFF ^ 0x43 ^ mode
```

For modes 8–15 (Powered Up extended modes), send EXT_MODE first:
```
EXT_MODE: [0x46, 0x08, checksum]   // 0x46 = CMD|SIZE_1|EXT_MODE(6)
SELECT:   [0x43, mode & 0x07, checksum]
```

### Examples

| Action | Bytes |
|--------|-------|
| Select mode 0 (DISTL) | `[0x43, 0x00, 0xBC]` |
| Select mode 3 (LIGHT write) | `[0x43, 0x03, 0xBF]` |
| Select mode 5 (RGB_I) | `[0x43, 0x05, 0xB9]` |
| Select mode 7 (SHSV) | `[0x43, 0x07, 0xBB]` |

---

## Data Write (Host→Sensor) — e.g., Setting Lights

For Powered Up devices, writing data always sends **EXT_MODE prefix** first:

```
1. EXT_MODE command:  [0x46, 0x00, 0xB9]   (ext_mode=0 for modes<8)
2. DATA message:      [HEADER, data..., pad..., checksum]
```

The DATA header encodes the mode AND the padded payload size:
```
HEADER = DATA(0xC0) | SIZE_field | (mode & 0x07)
```

Payload is **padded to next power of 2** (1, 2, 4, 8, 16, 32 bytes).

### Color Sensor: Set 3 LEDs (mode 3, 3×int8, padded to 4)

To set RGB = [R, G, B]:
```
EXT_MODE: [0x46, 0x00, 0xB9]
DATA:     [0xD3, R, G, B, 0x00, checksum]
           │          │    │
           │          │    └─ pad to 4 bytes
           │          └─ 3 brightness values (0-100 mapped to int8)
           └─ DATA(0xC0) | SIZE_4(0x10) | mode_3(0x03) = 0xD3
```

**Example: All white (100, 100, 100):**
```
EXT_MODE: [0x46, 0x00, 0xB9]
DATA:     [0xD3, 0x64, 0x64, 0x64, 0x00, chk]
checksum = 0xFF ^ 0xD3 ^ 0x64 ^ 0x64 ^ 0x64 ^ 0x00 = 0x68
Full:     [0xD3, 0x64, 0x64, 0x64, 0x00, 0x68]
```

**Example: Lights OFF (0, 0, 0):**
```
EXT_MODE: [0x46, 0x00, 0xB9]
DATA:     [0xD3, 0x00, 0x00, 0x00, 0x00, 0x2C]
```

### Ultrasonic Sensor: Set 4 LEDs (mode 5, 4×int8, exact 4)

To set 4 LEDs = [L1, L2, L3, L4]:
```
EXT_MODE: [0x46, 0x00, 0xB9]
DATA:     [0xD5, L1, L2, L3, L4, checksum]
           └─ DATA(0xC0) | SIZE_4(0x10) | mode_5(0x05) = 0xD5
```

**Example: All on (100,100,100,100):**
```
[0xD5, 0x64, 0x64, 0x64, 0x64, 0x6E]
```

---

## Data Read (Sensor→Host) — Distance / Color

Sensor sends DATA messages continuously at ~10-50Hz depending on mode.

### Ultrasonic Distance (mode 0, 1×int16_t)

```
[HEADER, low_byte, high_byte, checksum]

HEADER = DATA(0xC0) | SIZE_2(0x08) | mode_0(0x00) = 0xC8
```

**Example: 150mm:**
```
[0xC8, 0x96, 0x00, checksum]
distance = (int16_t)(0x0096) = 150
```

**Example: 2000mm (max / nothing detected):**
```
[0xC8, 0xD0, 0x07, checksum]
distance = (int16_t)(0x07D0) = 2000
```

### Color Sensor RGB_I (mode 5, 4×int16_t = 8 bytes)

```
[HEADER, R_lo, R_hi, G_lo, G_hi, B_lo, B_hi, I_lo, I_hi, checksum]

HEADER = DATA(0xC0) | SIZE_8(0x18) | mode_5(0x05) = 0xDD
```

Each value is int16_t little-endian, range 0–1024.

### Color Sensor SHSV (mode 7, 4×int16_t = 8 bytes)

```
HEADER = DATA(0xC0) | SIZE_8(0x18) | mode_7(0x07) = 0xDF
```

Payload: [S_lo, S_hi, H_lo, H_hi, SV_lo, SV_hi, V_lo, V_hi]

---

## Keepalive / NACK

Host MUST send `0x02` (NACK) to sensor every **≤100ms** or sensor resets.

In Pybricks: `EV3_UART_DATA_KEEP_ALIVE_TIMEOUT = 100ms`

The NACK also serves as the "I'm alive, keep sending data" signal. If the host doesn't send NACK within 100ms, the sensor enters error state and the LUMP link dies.

**Our RTIC firmware: This is the 250ms keepalive window we track.** The 100ms is the host→sensor interval. The sensor times out at roughly 250ms without NACK.

---

## Mode-Specific Timing (from device.c)

| Sensor | Mode | Stale Data Delay | Notes |
|--------|------|:----------------:|-------|
| Color (61) | LIGHT (3, write) | 0ms | Immediate |
| Color (61) | all read modes | 30ms | Wait 30ms after mode switch before data is valid |
| Ultrasonic (62) | LIGHT (5, write) | 0ms | Immediate |
| Ultrasonic (62) | all read modes | 50ms | Wait 50ms after mode switch |
| Any sensor | data_set_delay | 10ms | Minimum gap between successive writes |

---

## Color Sensor Modes

| Mode | Name  | Direction | Format | Description |
|:----:|-------|:---------:|--------|-------------|
| 0 | COLOR | read | 1×int8  | Discrete color ID |
| 1 | REFLT | read | 1×int8  | Reflection % |
| 2 | AMBI  | read | 1×int8  | Ambient light % |
| 3 | LIGHT | **write** | 3×int8 | LED brightness [R, G, B] |
| 4 | RREFL | read | 2×int16 | Raw reflection |
| 5 | RGB_I | read | 4×int16 | **Raw R, G, B, Intensity** (our primary mode) |
| 6 | HSV   | read | 3×int16 | Hue, Saturation, Value |
| 7 | SHSV  | read | 4×int16 | Surface HSV + ambient |
| 8 | DEBUG | read | 2×int16 | Debug data |
| 9 | CALIB | read | 7×int16 | Calibration data |

---

## Ultrasonic Sensor Modes

| Mode | Name  | Direction | Format | Description |
|:----:|-------|:---------:|--------|-------------|
| 0 | DISTL | read | 1×int16 | **Distance (mm), long range** — our primary mode |
| 1 | DISTS | read | 1×int16 | Distance (mm), short range |
| 2 | SINGL | read | 1×int16 | Single-shot distance |
| 3 | LISTN | read | 1×int8  | Ultrasonic presence detection |
| 4 | TRAW  | read | 1×int32 | Raw time-of-flight |
| 5 | LIGHT | **write** | 4×int8 | **LED brightness [L1, L2, L3, L4]** |
| 6 | PING  | read | 1×int8  | Unknown |
| 7 | ADRAW | read | 1×int16 | Raw ambient |
| 8 | CALIB | read | 7×int16 | Calibration data |

---

## How Pybricks Turns On the Color Sensor Light

**Key insight for RTIC:** The color sensor light is automatically ON when reading in certain modes. The light control is a separate WRITE mode (mode 3).

### Pybricks flow for `color.lights.on([R, G, B])`:

1. **MicroPython** calls `pb_type_device_set_data(sensor, mode=3, data=[R,G,B], size=3)`
2. **pbio** calls `pbio_port_lump_set_mode_with_data(lump_dev, mode=3, data, 3)`
3. This does:
   - `pbio_port_lump_set_mode(lump_dev, 3)` — requests mode switch to LIGHT
   - `pbio_port_lump_request_data_set(lump_dev, 3, data, 3)` — queues data write
4. **Send thread** picks up the mode switch request:
   - Sends: `[0x43, 0x03, 0xBF]` (SELECT mode 3)
5. Send thread sees queued data (once mode == 3):
   - Sends EXT_MODE: `[0x46, 0x00, 0xB9]`
   - Sends DATA: `[0xD3, R, G, B, 0x00, checksum]`
6. After write, `is_ready()` waits `data_set_delay` (10ms) before returning success

### For reading after lights are set:
When switching back to RGB_I mode 5:
- Sends: `[0x43, 0x05, 0xB9]` (SELECT mode 5)
- Waits 30ms stale data delay
- Then reads incoming DATA messages from sensor

**Important:** In RGB_I mode (5), the sensor's LEDs are ON by default (white light for reflection measurement). The LIGHT mode (3) is for custom LED colors. Switching to SHSV mode (7) turns the LEDs OFF (ambient measurement).

### For RTIC firmware:
- Mode 5 already has light ON — no extra command needed for basic operation
- Only use mode 3 LIGHT write if you need custom LED colors
- Always send NACK keepalive ≤100ms
- After mode switch, discard data for the stale_data_delay period

---

## Complete LUMP Session Example: Read Distance

```
// After handshake at 115200 baud:

Host → Sensor: [0x43, 0x00, 0xBC]     // SELECT mode 0 (DISTL)
Host → Sensor: [0x02]                   // NACK keepalive

// ~50ms later, sensor begins streaming:
Sensor → Host: [0xC8, 0x96, 0x00, xx]  // DATA: distance=150mm
Host → Sensor: [0x02]                   // NACK (within 100ms)
Sensor → Host: [0xC8, 0x97, 0x00, xx]  // DATA: distance=151mm
Host → Sensor: [0x02]                   // NACK
// ... continues ...
```

## Complete LUMP Session Example: Set Color Sensor Light

```
// Currently in mode 5 (RGB_I), want to set lights to green:

Host → Sensor: [0x43, 0x03, 0xBF]                 // SELECT mode 3 (LIGHT)
Host → Sensor: [0x02]                               // NACK keepalive
// Wait for sensor to acknowledge mode switch (via DATA msg with mode=3)

Host → Sensor: [0x46, 0x00, 0xB9]                 // EXT_MODE = 0
Host → Sensor: [0xD3, 0x00, 0x64, 0x00, 0x00, xx] // DATA: R=0, G=100, B=0

Host → Sensor: [0x02]                               // NACK keepalive
// 10ms data_set_delay

// Switch back to RGB_I:
Host → Sensor: [0x43, 0x05, 0xB9]                 // SELECT mode 5 (RGB_I)
// 30ms stale data delay, then read stream resumes
```

---

## Differences from Our RTIC Implementation

| Aspect | Pybricks | Our RTIC |
|--------|----------|----------|
| Keepalive interval | 100ms (sends NACK) | sensor_poll at ~50Hz (20ms) |
| Keepalive byte | 0x02 (NACK) | 0x02 (NACK) — same |
| Mode switch | SELECT cmd + wait for data | Same approach |
| Light control | Mode 3 write with EXT_MODE prefix | Need to add EXT_MODE prefix |
| Stale data delay | 30-50ms per sensor type | Need to implement |
| Handshake | Full SYNC→TYPE→INFO→ACK | Same but verify baud change timing |

---

*Source files:*
- `lib/pbio/src/port_lump.c` — Main LUMP protocol (1250+ lines)
- `lib/lego/lego/lump.h` — Protocol constants
- `lib/lego/lego/device.h` — Mode definitions
- `lib/lego/device.c` — Timing delays
- `pybricks/common/pb_type_lightarray.c` — Light write API
- `pybricks/common/pb_type_device.c` — Mode switch + data access layer
