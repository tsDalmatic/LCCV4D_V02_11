# LCCV4D V02.11

Firmware for the Dalmatic LCCV4D controller, targeting the STM32F091VC.

## Build

Requirements:

- CMake 3.22 or newer
- Ninja
- GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`)
- PowerShell (used to calculate the Class-B firmware CRC)

Configure and build the Debug preset:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Build outputs are written to `build/Debug`, including the ELF, Intel HEX, and
raw binary firmware images.

Use **Terminal > Run Task > Build and Flash** to build and program the target
through ST-LINK. The equivalent command-line programmer invocation is stored in
`.vscode/tasks.json`.

## Class-B checksum

The reference CRC is the `_Check_Sum` word at the end of
`cmake/startup_stm32f091xc.s`. It must match `crc_result` at the line
`crc_result = CRC->DR` in
`STM32_SelfTest_Library/src/stm32fxx_STLstartup.c`.

After changing firmware:

1. Set a breakpoint on `crc_result = CRC->DR` before starting the debug
   session.
2. Start **Cortex Debug**. It stops first at `STL_StartUp`; continue execution.
   The Class-B watchdog test intentionally performs IWDG and WWDG resets before
   reaching the CRC calculation, so keep the breakpoint enabled through those
   resets.
3. Copy the hexadecimal value of `crc_result` into `_Check_Sum` in the GNU
   startup file.
4. Rebuild. The post-build check uses the same `__REV` word order as the target
   and rejects a stale checksum.

The GNU startup file contains separate values for the Debug and Release images:

- Debug: `0x2F845954`
- Release: `0xC3E042DC`

## Keil and CubeProgrammer

The Keil project remains available at `MDK-ARM/V4D-V02_11.uvprojx`. It uses
`MDK-ARM/startup_stm32f091xc.s` and
`STM32_SelfTest_Library/Boot_Flash_ClassB.sct`; its checksum is independent of
the GNU Debug and Release checksums because the linked image is different.

The supplied Class-B CPU-test object files use Keil's register convention and
overwrite R4-R11 during their tests. CMake adds GCC-only preserving wrappers so
that GCC's frame pointer and other callee-saved registers remain valid. Keil
continues to call the original routines directly.

STM32CubeProgrammer can program any of these CMake artifacts directly:

- `build/Debug/V4D-V02_11.elf`
- `build/Debug/V4D-V02_11.hex`
- `build/Debug/V4D-V02_11.bin`

The CubeMX project metadata (`V4D-V02_11.ioc` and `.mxproject`) and the complete
STM32 driver tree are intentionally retained for compatibility with the other
development workflows.
