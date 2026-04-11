[← Main README](../README.md) · [User Manual](../USER_MANUAL.md) · [Reference Manual](../REFERENCE_MANUAL.md) · [API Reference](../spike-hub-api/README.md) · [RAM Demos](../examples/hub-ram-demos/README.md) · [Helper Tools](../helper-tools/README.md) · [Dev Notes](../dev_notes/)

---

Can QEMU do realtime clock?
Yes and no. QEMU emulates SysTick faithfully — the counter counts down, interrupts fire, Mono::delay(100.millis()).await completes. But it runs at virtual CPU speed, not wall-clock time. A 100ms delay completes near-instantly. This is fine for functional testing (does the code path work?) but not for real-time timing verification.

MPU enforcement gap
QEMU 8.2's netduinoplus2 machine does not enforce MPU access permissions. The MPU registers are read/write (the config code works) but writes to protected regions don't fault. This means the guard band MemManage → reset path cannot be end-to-end tested in QEMU. It can only be verified on real hardware.

The stack painting and high-water mark features work perfectly in QEMU since they don't depend on the MPU — they're pure memory read/write.

For real firmware integration
When porting to sandbox.rs on the real hub, the guard bands go into the existing mpu_configure() as regions 5 and 6 (3 of 8 currently unused). The MemManage handler needs a branch: if SANDBOXED is false → MSP fault → system reset.