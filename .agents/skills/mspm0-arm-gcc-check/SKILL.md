---
name: mspm0-arm-gcc-check
description: Use when validating MSPM0G3507 C source locally with arm-none-eabi-gcc, especially when the active Keil/MDK project cannot be built from the command line. Covers extracting MDK defines/include paths, Cortex-M0+ syntax checks, warning policy, compile-time configuration matrices, and the boundary between GCC source validation and an actual AC6/Keil build.
---

# MSPM0 ARM GCC Check

Use the installed `arm-none-eabi-gcc` as a fast source-level validation fallback for this MSPM0G3507 Keil project.

## Boundaries

- Treat `project/mdk/SeekFree_MSPM0G3507_Device_Library.uvprojx` as the source of truth for preprocessor defines and include paths.
- Define `__MSPM0G3507__`; omitting it prevents TI device headers from declaring `SPI1`, `GPIO_Regs`, `__STATIC_INLINE`, and other device symbols.
- Use `-mcpu=cortex-m0plus -mthumb` for the target core.
- Use `-fsyntax-only`; do not create or commit GCC objects.
- Do not claim this validates ARMCLANG/AC6 linking, scatter loading, startup assembly, SysConfig regeneration, Keil Before Make commands, flashing, or target-board behavior.
- Keep Keil AC6 V6.23 and target-board verification as the authoritative final checks.

## Warning Policy

Use:

```text
-Wall -Wextra -Werror -Wno-unused-parameter
```

The only general suppression is `-Wno-unused-parameter`, required because bundled TI DriverLib inline functions contain known unused parameters. Do not suppress other warnings merely to make a check pass.

## Include Paths

Use the active target's `<IncludePath>` from the MDK project. For the current repository:

```bash
INCLUDES='-Ilibraries/sdk/third_party/CMSIS/Core/Include -Ilibraries/sdk -Ilibraries/sdk/ti_config -Ilibraries/zf_common -Ilibraries/zf_components -Ilibraries/zf_device -Ilibraries/zf_driver -Iproject/user/inc -Iproject/code -Iproject/code/motor_control -Iproject/code/v8_protocol -Iproject/code/speed_sampling -Iproject/code/speed_sampling/speed_sampling -Iproject/code/ec11_encoder -Iproject/code/system_tick -Iproject/code/persist_config -Iproject/code/motor_menu -Iproject/wegui/Lcd_Driver -Iproject/wegui/Lcd_Port -Iproject/wegui/Lcd_WeGui'
```

Run commands from the repository root. Re-read the `.uvprojx` before reuse if the project layout changes.

## Source Check

Validate the touched translation units together when their headers or declarations interact:

```bash
arm-none-eabi-gcc \
  -std=gnu11 -mcpu=cortex-m0plus -mthumb \
  -D__MSPM0G3507__ \
  -fsyntax-only -Wall -Wextra -Werror -Wno-unused-parameter \
  $INCLUDES \
  libraries/zf_device/zf_device_bmi088.c \
  project/user/src/main.c \
  project/code/system_tick/system_tick.c \
  project/code/v8_protocol/v8_protocol.c
```

An empty output with exit status 0 is a pass.

## Compile-Time Matrix

When behavior is selected by macros, compile every supported branch. Example for the BMI088 diagnostic stages:

```bash
for stage in 1 2 3 4 5 6; do
  arm-none-eabi-gcc \
    -std=gnu11 -mcpu=cortex-m0plus -mthumb \
    -D__MSPM0G3507__ -DBMI088_DIAG_STAGE=$stage \
    -fsyntax-only -Wall -Wextra -Werror -Wno-unused-parameter \
    $INCLUDES \
    libraries/zf_device/zf_device_bmi088.c || exit 1
done
```

Also compile important disabled paths when editing conditional diagnostics:

```bash
arm-none-eabi-gcc \
  -std=gnu11 -mcpu=cortex-m0plus -mthumb \
  -D__MSPM0G3507__ -DBMI088_DIAG_MARKER_ENABLED=0 \
  -fsyntax-only -Wall -Wextra -Werror -Wno-unused-parameter \
  $INCLUDES \
  project/user/src/main.c \
  project/code/system_tick/system_tick.c
```

## Companion Checks

Run source formatting validation separately:

```bash
git diff --check
```

For SysConfig and project-shape validation, use the existing `mspm0-ccs` skill and its `check_syscfg.py` helper. Do not edit generated `ti_msp_dl_config.c` or `ti_msp_dl_config.h` to satisfy GCC.

## Failure Interpretation

- Missing `SPI1`, `GPIO_Regs`, `IRQn_Type`, or `__STATIC_INLINE`: first confirm `-D__MSPM0G3507__` and the CMSIS/SDK include paths.
- Errors only from a compile-time branch: rerun the exact matrix value and fix that branch rather than weakening warnings.
- TI DriverLib unused-parameter diagnostics: retain `-Wno-unused-parameter`; do not patch generated or vendor headers.
- GCC passes but Keil fails: treat the Keil diagnostic as authoritative and fix compiler-specific or project-integration issues.

## Reporting

Report this as "Cortex-M0+ ARM GCC syntax and warning check passed," not "firmware build passed." Explicitly state when Keil build, link, download, or hardware validation was not performed.
