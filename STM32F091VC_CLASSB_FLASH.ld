ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)       : ORIGIN = 0x08000000, LENGTH = 256K
  RAM_BUF (xrw)    : ORIGIN = 0x20000000, LENGTH = 0x20
  RAM_PNT (xrw)    : ORIGIN = 0x20000020, LENGTH = 0x10
  CLASSB (xrw)     : ORIGIN = 0x20000030, LENGTH = 0x28
  CLASSB_INV (xrw) : ORIGIN = 0x20000058, LENGTH = 0x28
  RAM (xrw)        : ORIGIN = 0x20000080, LENGTH = 0x3B70
  STACK (xrw)      : ORIGIN = 0x20003BF0, LENGTH = 0x410
}

_estack = ORIGIN(STACK) + LENGTH(STACK);
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text .text*)
    *(.glue_7 .glue_7t)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .rodata : { . = ALIGN(4); *(.rodata .rodata*) . = ALIGN(4); } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data .data*)
    *(.RamFunc .RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss .bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM

  .run_time_ram_buf (NOLOAD) : { KEEP(*(.run_time_ram_buf)) } > RAM_BUF
  .run_time_ram_pnt (NOLOAD) : { KEEP(*(.run_time_ram_pnt)) } > RAM_PNT
  .class_b_ram (NOLOAD) : { KEEP(*(.class_b_ram)) } > CLASSB
  .class_b_ram_rev (NOLOAD) : { KEEP(*(.class_b_ram_rev)) } > CLASSB_INV
  .stack_bottom (NOLOAD) : { KEEP(*(.stack_bottom)) } > STACK

  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . += _Min_Heap_Size;
    . = ALIGN(8);
  } > RAM

  .checksum ALIGN(LOADADDR(.data) + SIZEOF(.data), 64) :
  {
    KEEP(*(.checksum))
  } > FLASH

  /DISCARD/ : { libc.a:* (*) libm.a:* (*) libgcc.a:* (*) }
}

ASSERT(SIZEOF(.run_time_ram_buf) <= LENGTH(RAM_BUF), "Class-B RAM buffer overflow")
ASSERT(SIZEOF(.run_time_ram_pnt) <= LENGTH(RAM_PNT), "Class-B RAM pointer overflow")
ASSERT(SIZEOF(.class_b_ram) <= LENGTH(CLASSB), "Class-B RAM overflow")
ASSERT(SIZEOF(.class_b_ram_rev) <= LENGTH(CLASSB_INV), "Class-B inverse RAM overflow")
ASSERT(SIZEOF(.stack_bottom) <= 0x10, "Stack guard overflow")
ASSERT(SIZEOF(.checksum) == 4, "Class-B checksum must be exactly one 32-bit word")
