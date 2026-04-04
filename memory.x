/* Linker script for LEGO SPIKE Prime Hub
 * MCU: STM32F413VGT6 (1 MB Flash, 320 KB SRAM)
 *
 * Flash layout:
 *   0x08000000..0x08008000  LEGO DFU bootloader (32 KB, factory, do NOT erase)
 *   0x08008000..0x08100000  Our RTIC firmware   (992 KB)
 *
 * RAM layout:
 *   0x20000000..0x20040000  SRAM1 (256 KB) — stack, .bss, .data
 *   0x20040000..0x20050000  SRAM2 ( 64 KB) — upload / demo buffer
 *   DFU magic word at 0x2004FFF0 (last 16 bytes reserved)
 *
 * The LEGO bootloader sets VTOR and jumps to 0x08008000.
 * We place our vector table there.
 */
MEMORY
{
    FLASH : ORIGIN = 0x08008000, LENGTH = 992K
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
    SRAM2 : ORIGIN = 0x20040000, LENGTH = 64K
}

SECTIONS
{
    .sram2 (NOLOAD) : ALIGN(4)
    {
        *(.sram2 .sram2.*)
    } > SRAM2
} INSERT AFTER .bss;
