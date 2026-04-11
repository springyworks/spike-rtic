[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---



14:25:54:~/projects/rust/spike-rtic +:dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D target/spike-rtic.bin
dfu-util 0.11

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2021 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

dfu-util: Warning: Invalid DFU suffix signature
dfu-util: A valid DFU suffix will be required in a future dfu-util release
Opening DFU capable USB device...
Device ID 0694:0011
Device DFU version 011a
Claiming USB DFU Interface...
Setting Alternate Interface #0 ...
Determining device status...
DFU state(2) = dfuIDLE, status(0) = No error condition is present
DFU mode device DFU version 011a
Device returned transfer size 2048
DfuSe interface name: "LEGO LES HUB "
Downloading element to address = 0x08008000, size = 140220
Erase           [=========================] 100%       140220 bytes
Erase    done.
Download        [=========================] 100%       140220 bytes
Download done.
File downloaded successfully
Submitting leave request...
Transitioning to dfuMANIFEST state

# connect to hub , smart
14:26:54:~/projects/rust/spike-rtic +:python3 helper-tools/hub_state.py gdb examples/hub-ram-demos/target/thumbv7em-none-eabihf/release/examples/gdb_simple
  Hub in shell mode on /dev/ttyACM1. Sending 'gdb' command...
  RSP entered. Response: b'gdb\r\nGDB stub active. Connect with:\r\n  arm-none-eabi-gdb -ex "target remote /dev/ttyACM0"\r\nType Ctrl-C three times rapidly to exit GDB mode.\r\nspike> '
  Launching: gdb-multiarch -q examples/hub-ram-demos/target/thumbv7em-none-eabihf/release/examples/gdb_simple -ex set pagination off -ex set architecture arm -ex target remote /dev/ttyACM1
Reading symbols from examples/hub-ram-demos/target/thumbv7em-none-eabihf/release/examples/gdb_simple...
The target architecture is set to "arm".
Remote debugging using /dev/ttyACM1
0x00000074 in ?? ()
(gdb)  info registers
r0             0x0                 0
r1             0x0                 0
r2             0x0                 0
r3             0x0                 0
r4             0x0                 0
r5             0x0                 0
r6             0x0                 0
r7             0x0                 0
r8             0x0                 0
r9             0x0                 0
r10            0x0                 0
r11            0x0                 0
r12            0x0                 0
sp             0x2003ed98          0x2003ed98
lr             0x74                116
pc             0x74                0x74
cpsr           0x60000053          1610612819
(gdb) 