# Repository Guidance

Single STM32H750VBT6 firmware application using STM32CubeMX/HAL and a CMake ARM GCC build.

## Skills

Load the relevant skill before specialized work:

| Skill                                                                  | Use when                                              |
| ---------------------------------------------------------------------- | ----------------------------------------------------- |
| [arm-cortex-expert](.agents/skills/arm-cortex-expert/SKILL.md)         | STM32/Cortex-M firmware or driver work                |
| [embedded-systems](.agents/skills/embedded-systems/SKILL.md)           | MCU, interrupt, DMA, timing, RTOS, or power work      |
| [embedded-debug](.agents/skills/embedded-debug/SKILL.md)               | Hard faults, crashes, register dumps, or stack traces |
| [memory-audit](.agents/skills/memory-audit/SKILL.md)                   | Stack, heap, allocation, or memory-risk analysis      |
| [timing-analysis](.agents/skills/timing-analysis/SKILL.md)             | ISR, DMA, or real-time timing analysis                |
| [ano-v8-host](.agents/skills/ano-v8-host/SKILL.md)                     | ANO V8 host protocol or telemetry work                |
| [v8-chinese-convention](.agents/skills/v8-chinese-convention/SKILL.md) | V8 protocol or motor-control naming work              |

## Structure

- `Core/Src/main.c` is the application entrypoint. Startup calls `MPU_Config()`, `HAL_Init()`, `SystemClock_Config()`, then `App_Peripheral_Init()`; the main loop calls `App_Process()`.
- `Core/` and `Drivers/` contain CubeMX/HAL-generated or vendor code. `STM32H750VBT6_NUEDC.ioc` is the CubeMX peripheral configuration.
- Application drivers and control logic live under `BSP/Hardware` and `BSP/Software`.
- The root `CMakeLists.txt` explicitly selects the BSP subdirectories compiled into the firmware; adding a new BSP module requires adding its source glob there.

## Build

- `arm-none-eabi-gcc`, Ninja, and CMake 3.22+ must be available on `PATH`.
- Configure and build with the ARM presets: `cmake --preset MinSizeRel` then `cmake --build --preset MinSizeRel`.
- Other presets are `Debug`, `RelWithDebInfo`, and `Release`; use the same preset name for configure and build.
- The toolchain targets Cortex-M7 hard-float and links `stm32h750vbtx_flash.ld`; do not use the host compiler.
- Build output is under `build/<preset>/` and is ignored. If the checkout path changes, configure a fresh preset directory because CMake caches absolute paths.

## Change Boundaries

- Treat `STM32H750VBT6_NUEDC.ioc` and CubeMX-generated sections/files as configuration-owned. Regenerate deliberately and review generated diffs before keeping manual changes.
- Keep application behavior in `Core/Src/main.c` and BSP modules; do not add a second entrypoint.
- `CMakeLists.txt` uses `GLOB_RECURSE ... CONFIGURE_DEPENDS` only for the listed BSP directories, not for every file under `BSP/`.