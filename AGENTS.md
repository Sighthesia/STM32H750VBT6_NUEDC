# Repository Guidance

This repository contains a single STM32H750VBT6 firmware application.

## Skills

Load the relevant project skill before specialized work:

| Skill | Use when |
| --- | --- |
| [arm-cortex-expert](.agents/skills/arm-cortex-expert/SKILL.md) | STM32/Cortex-M firmware or driver work |
| [embedded-systems](.agents/skills/embedded-systems/SKILL.md) | MCU, interrupt, DMA, timing, RTOS, or power work |
| [embedded-debug](.agents/skills/embedded-debug/SKILL.md) | Hard faults, crashes, register dumps, or stack traces |
| [memory-audit](.agents/skills/memory-audit/SKILL.md) | Stack, heap, allocation, or memory-risk analysis |
| [timing-analysis](.agents/skills/timing-analysis/SKILL.md) | ISR, DMA, or real-time protocol timing analysis |
| [v8-chinese-convention](.agents/skills/v8-chinese-convention/SKILL.md) | V8 protocol or motor-control naming work |
| [mspm0-arm-gcc-check](.agents/skills/mspm0-arm-gcc-check/SKILL.md) | MSPM0 GCC source validation only |
| [mspm0-ccs](.agents/skills/mspm0-ccs/SKILL.md) | MSPM0 CCS/Keil/SysConfig work only |

## Project Shape

- This is a single STM32H750VBT6 firmware project; the application entrypoint is `Core/Src/main.c`.
- `main()` performs MPU/HAL/clock startup, then calls `App_Peripheral_Init()`, `App_Menu_Init()`, `App_Control_Init()`, and repeatedly runs `App_Process()`.
- CubeMX peripheral configuration is recorded in `STM32H750VBT6_NUEDC.ioc`; generated application and peripheral sources are under `Core/Src`.
- Application drivers live under `BSP/Hardware` and `BSP/Software`; top-level CMake automatically collects `BSP/**/*.c` and derives include directories from BSP headers.

## Build

- Use the ARM toolchain preset, not the host compiler: `cmake --preset MinSizeRel` then `cmake --build --preset MinSizeRel`.
- Other configured build presets are `Debug`, `RelWithDebInfo`, and `Release`.
- The preset uses Ninja and `cmake/gcc-arm-none-eabi.cmake`; it expects `arm-none-eabi-gcc` on `PATH` and sets Cortex-M7 hard-float flags.
- `CMakeLists.txt` enables C11, exports `compile_commands.json`, and links the STM32H750 linker script through the toolchain configuration.
- Build directories embed absolute source paths. If a checkout moves or the cache points elsewhere, configure a fresh directory or remove the stale build directory before building.

## Change Boundaries

- Treat `STM32H750VBT6_NUEDC.ioc` and CubeMX-generated sections/files as configuration-owned; regenerate deliberately and review generated diffs before hand edits.
- Keep application behavior in `Core/Src/main.c` and BSP modules; do not add a second application entrypoint.
- There is no repository README, CI workflow, or test suite defining additional commands; do not invent test requirements.
