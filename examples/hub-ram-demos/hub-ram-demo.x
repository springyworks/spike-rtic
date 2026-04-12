/* Linker script for RAM demos on LEGO SPIKE Prime Hub.
 *
 * The demo is uploaded to SRAM2 and executed via the monitor's
 * `go` command.  Entry point: _start(api: *const MonitorApi) -> u32
 *
 * SRAM2 is 64 KB at 0x20040000.  Last 16 bytes reserved for DFU magic.
 * Stack is provided by the caller (monitor firmware) — no vector table.
 */
ENTRY(_start)

MEMORY
{
    RAM : ORIGIN = 0x20040000, LENGTH = 0xFFF0
}

SECTIONS
{
    /* Optional demo header — privileged demos place PRIV_MAGIC here.
     * Firmware checks the first word before launching; if it matches
     * PRIV_MAGIC and the user typed `go` (sandboxed), launch is refused.
     * Non-privileged demos omit this section so _start stays at offset 0.
     */
    .demo_header :
    {
        KEEP(*(.demo_header))
    } > RAM

    .text :
    {
        KEEP(*(.text._start))
        *(.text .text.*)
    } > RAM

    .rodata : ALIGN(4)
    {
        *(.rodata .rodata.*)
    } > RAM

    .data : ALIGN(4)
    {
        *(.data .data.*)
    } > RAM

    .bss (NOLOAD) : ALIGN(4)
    {
        *(.bss .bss.*)
    } > RAM

    /DISCARD/ :
    {
        *(.ARM.exidx .ARM.exidx.*)
        *(.ARM.attributes)
    }
}
