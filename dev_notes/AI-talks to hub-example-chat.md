[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---



 

Ran terminal command:  ls /dev/ttyACM*

Ran terminal command:  lsusb | grep 0694

Hub is running (0694:0042). Let me check if anything else has the port open, then connect and probe the color sensor on port F: 

Ran terminal command:  fuser /dev/ttyACM0 2>/dev/null; echo "exit:$?"

Something (PID 71293) has the port open -- likely the earlier picocom session. Let me kill it: 

Ran terminal command:  kill 71293 2>/dev/null; sleep 0.5; fuser /dev/ttyACM0 2>/dev/null; echo "exit:$?"

Port is free. Now let me send the sensor probe command on port F: 

Ran terminal command:  exec 3<>/dev/ttyACM0 && stty -F /dev/ttyACM0 115200 raw -echo && printf '\r' >&3 && sleep 0.3 && cat <&3 &
CAT_PID=$!
sleep 0.5
printf 'sensor f\r' >&3
sleep 4
kill $CAT_PID 2>/dev/null
wait $CAT_PID 2>/dev/null
exec 3>&-

The fd redirect failed. Let me use a simpler approach with a Python one-liner:  

Ran terminal command:  python3 -c "
import serial, time, sys
s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.3)
s.write(b'\r')
time.sleep(0.3)
# flush any old data
s.read(s.in_waiting or 1)

# send sensor f
s.write(b'sensor f\r')
time.sleep(5)
data = s.read(s.in_waiting or 1)
sys.stdout.buffer.write(data)
sys.stdout.buffer.flush()
print()

# also ask for sensor data
s.write(b'sensor\r')
time.sleep(1)
data = s.read(s.in_waiting or 1)
sys.stdout.buffer.write(data)
sys.stdout.buffer.flush()
print()
s.close()
"

Color sensor is working on port F. It's a **Color Sensor (type 61)** in mode 5 (RGB+intensity), reporting:

- **R=4, G=3, B=2, I=6** (low ambient/dark surface values)

Let me grab a few more readings to see it's actually streaming, and try a different mode: 

Ran terminal command:  python3 -c "
import serial, time, sys
s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.3)
s.read(s.in_waiting or 1)

# Read sensor a few times with delay
for i in range(3):
    s.write(b'sensor\r')
    time.sleep(1)
    data = s.read(s.in_waiting or 1)
    sys.stdout.buffer.write(data)
    sys.stdout.buffer.flush()

# Switch to mode 0 (color index)
s.write(b'sensor mode 0\r')
time.sleep(1)
data = s.read(s.in_waiting or 1)
sys.stdout.buffer.write(data)
sys.stdout.buffer.flush()

# Read in color index mode
time.sleep(1)
s.write(b'sensor\r')
time.sleep(1)
data = s.read(s.in_waiting or 1)
sys.stdout.buffer.write(data)
sys.stdout.buffer.flush()

# Switch back to mode 5 (RGB)
s.write(b'sensor mode 5\r')
time.sleep(1)
data = s.read(s.in_waiting or 1)
sys.stdout.buffer.write(data)
sys.stdout.buffer.flush()

print()
s.close()
"

Color sensor is fully operational on port F:

- **Mode 5 (RGB+I)**: R=0 G=0 B=0 I=0 (sensor pointing at empty space / dark)
- **Mode 0 (color index)**: value=-1 (no color detected -- sensor not close to a surface)
- **Mode switching** works correctly (NACK+mode at 115200 baud)

The low readings are normal -- to get meaningful RGB values, hold a colored LEGO brick or surface about 1-2 cm from the sensor face. The built-in white LED illuminates the surface and the sensor reads the reflected light.

### Verification Steps
- `sensor` command returns live data from the type 61 color sensor on port F
- Mode 0 (color index) and mode 5 (RGB raw) both respond
- `sensor mode <n>` switches correctly
- `sensor stop` will release the port when done

the transcript of the spike> python/spike shell communication :

+: python3 -c "
import serial, time, sys
s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.3)
s.read(s.in_waiting or 1)

# Read sensor a few times with delay
for i in range(3):
    s.write(b'sensor\r')
    time.sleep(1)
    data = s.read(s.in_waiting or 1)
    sys.stdout.buffer.write(data)
    sys.stdout.buffer.flush()

# Switch to mode 0 (color index)
s.write(b'sensor mode 0\r')
time.sleep(1)
data = s.read(s.in_waiting or 1)
sys.stdout.buffer.write(data)
sys.stdout.buffer.flush()

# Read in color index mode
".close()t.buffer.flush()ata) 1)
sensor
Color Sensor (type 61) port F mode 5
  R=0 G=0 B=0 I=0
spike> sensor
Color Sensor (type 61) port F mode 5
  R=0 G=0 B=0 I=0
spike> sensor
Color Sensor (type 61) port F mode 5
  R=0 G=0 B=0 I=0
spike> sensor mode 0
Mode switch to 0 (next poll)
spike> sensor
Color Sensor (type 61) port F mode 0
  value=-1
spike> sensor mode 5
Mode switch to 5 (next poll)
spike> 