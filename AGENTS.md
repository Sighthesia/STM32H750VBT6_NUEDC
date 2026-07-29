# Repository Guidance

This repository contains a single STM32H750VBT6 firmware application.

## Skills

Load the relevant project skill before specialized work:

| Skill                                                                  | Use when                                                 |
| ---------------------------------------------------------------------- | -------------------------------------------------------- |
| [arm-cortex-expert](.agents/skills/arm-cortex-expert/SKILL.md)         | STM32/Cortex-M firmware or driver work                   |
| [ano-v8-host](.agents/skills/ano-v8-host/SKILL.md)                     | ANO V8 匿名上位机协议、参数/命令发现、遥测或波特率持久化 |
| [embedded-systems](.agents/skills/embedded-systems/SKILL.md)           | MCU, interrupt, DMA, timing, RTOS, or power work         |
| [embedded-debug](.agents/skills/embedded-debug/SKILL.md)               | Hard faults, crashes, register dumps, or stack traces    |
| [memory-audit](.agents/skills/memory-audit/SKILL.md)                   | Stack, heap, allocation, or memory-risk analysis         |
| [timing-analysis](.agents/skills/timing-analysis/SKILL.md)             | ISR, DMA, or real-time protocol timing analysis          |
| [v8-chinese-convention](.agents/skills/v8-chinese-convention/SKILL.md) | V8 protocol or motor-control naming work                 |
| [mspm0-arm-gcc-check](.agents/skills/mspm0-arm-gcc-check/SKILL.md)     | MSPM0 GCC source validation only                         |
| [mspm0-ccs](.agents/skills/mspm0-ccs/SKILL.md)                         | MSPM0 CCS/Keil/SysConfig work only                       |

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

## Azure A1 网表

## 1. 电源与地网络 (Power & Ground Nets)

| 网络名称 (Net Name) | 关联引脚及模块 (Connected Pins & Components)                           | 来源图页 |
| ------------------- | ---------------------------------------------------------------------- | -------- |
| **+5V**             | USB1(A4, A9, B4, B9)、H2(Pin 2)、AMS1117 输入、USB2TTL 模块 (H1 Pin 1) |

 | 图页 1

 |
| **3.3V** | MCU U4 VDD 引脚 (11, 27, 50, 75, 100)；H2(Pin 18, 20)；H3(SWD Pin 1)；QSPI Flash U6(Pin 8)；NAND U5(VCC)；外设模块 H5, H6, H7, H8, H10, U7, U8

 | 图页 1, 2, 3

 |
| **GND** | USB1(A1, B1, A12, B12)；H2(Pin 1, 17, 19)；MCU U4 VSS 引脚 (10, 26, 49, 74, 99)；U4 VSSA (19)；SWD H3(Pin 3)；外设模块公共接地端

 | 图页 1, 2, 3

 |
| **VBAT** | 电池 BT1、BAT54C 复合二极管 D2、MCU U4 Pin 6 (VBAT)

 | 图页 1, 2

 |
| **VCAP** | MCU U4 Pin 48 (VCAP_1)、Pin 73 (VCAP_2)，连接滤波电容 C21 (2.2µF), C24 (2.2µF) | 图页 2 |
| **VDDA / VREF+** | MCU U4 Pin 21 (VDDA)、Pin 20 (VREF+)，连接滤波电容 C16 (100nF), C17 (10µF) | 图页 2 |

---

## 2. 主控 MCU (U4: STM32H750VBT6) 核心网络

### 核心控制与时钟网络

* **NRST**: MCU U4 Pin 14 <-> 按键 SW2 <-> 下载接口 H3 Pin 5 <-> USB2TTL H1 Pin 5


* **BOOT0**: MCU U4 Pin 94 <-> 按键 SW3 <-> 下拉电阻 R5 (10k)
* **PH0-OSC_IN**: MCU U4 Pin 12 <-> 25MHz 晶振 X2 Pin 1 <-> 电容 C10 (16pF)
* **PH1-OSC_OUT**: MCU U4 Pin 13 <-> 25MHz 晶振 X2 Pin 2 <-> 电容 C13 (22pF)
* **PC14-OSC32_IN**: MCU U4 Pin 8 <-> 32.768kHz 晶振 X1 Pin 1 <-> 电容 C8 (16pF)
* **PC15-OSC32_OUT**: MCU U4 Pin 9 <-> 32.768kHz 晶振 X1 Pin 2 <-> 电容 C9 (16pF)

### 调试下载接口 (SWD)

* **PA13 (SWDIO)**: MCU U4 Pin 72 <-> 下载接口 H3 Pin 4
* **PA14 (SWCLK)**: MCU U4 Pin 76 <-> 下载接口 H3 Pin 2

---

## 3. 存储器网络 (Flash Memory Nets)

### QSPI Flash (U6: W25Q32JVSSIQ)

* **FLASH_CS (PB6)**: U4 Pin 92 <-> U6 Pin 1 (CS#)
* **FLASH_DO / IO1 (PB2)**: U4 Pin 36 <-> U6 Pin 2 (DO/IO1)
* **FLASH_WP / IO2 (PD6)**: U4 Pin 87 <-> U6 Pin 3 (WP#/IO2)
* **FLASH_DI / IO0 (PC9)**: U4 Pin 66 <-> U6 Pin 5 (DI/IO0)
* **FLASH_CLK (PC8)**: U4 Pin 65 <-> U6 Pin 6 (CLK)
* **FLASH_HOLD / IO3 (PD13)**: U4 Pin 60 <-> U6 Pin 7 (HOLD#/IO3)

### NAND Flash (U5: MT29F4G08ABADAWP:D)

* **数据总线 (I/O0 - I/O7)**:
* I/O0 (PD14 / Pin 61), I/O1 (PD15 / Pin 62), I/O2 (PD0 / Pin 81), I/O3 (PD1 / Pin 82)
* I/O4 (PE7 / Pin 37), I/O5 (PE8 / Pin 38), I/O6 (PE9 / Pin 39), I/O7 (PE10 / Pin 40)


* **控制总线**:
* **CLE**: PD11 (Pin 58) <-> U5 Pin 16
* **ALE**: PD12 (Pin 59) <-> U5 Pin 17
* **WE#**: PD5 (Pin 86) <-> U5 Pin 18
* **WP#**: PD7 (Pin 88) <-> U5 Pin 19
* **CE#**: PC8 (Pin 65) <-> U5 Pin 9
* **R/B#**: PD6 (Pin 87) <-> U5 Pin 7
* **RE#**: PD4 (Pin 85) <-> U5 Pin 8



---

## 4. 外设模块与接口网表 (Peripherals & Connectors)

### Type-C USB 接口 (USB1)



* **PA11 (USB_DN)**: USB1 Pin A7/B7 (DN1/DN2) <-> MCU U4 Pin 70


* **PA12 (USB_DP)**: USB1 Pin A6/B6 (DP1/DP2) <-> MCU U4 Pin 71


* **CC1 / CC2**: 分别连接至 Type-C 接口 A5 / B5 脚



### 板载串口/扩展模组接口

* **USB2TTL 模组 (H1)**: PB14 (TXD), PB15 (RXD), NRST, VCC, GND


* **EEPROM 模组 (H6)**: PB8 (SCL), PB9 (SDA), 3.3V, GND
* **SD Card 模组 (H7)**: SD_CS, SD_MOSI, SD_CLK, SD_MISO, 3.3V, GND
* **TFT LCD 模组 (H8)**: LCD_BL, LCD_CS, LCD_DC, LCD_RES, LCD_SDA, LCD_SCL
* **RS485 & CAN 模组 (H10)**: 485_RX, 485_TX, CAN_RX, CAN_TX, 3.3V, GND
* **WIFI/BT 模组 (U7: ESP8684-WROOM-04C)**: WIFI_RX, WIFI_TX, IO0~IO22 功能控制引脚
* **DCMI 摄像头接口 (FPC1 - 18P)**:
* 信号网络: OV_SCL(PB10), OV_SDA(PB11), OV_VSYNC(PB7), OV_HREF(PA4), OV_PCLK(PA6), OV_RST(NRST), OV_PWDN(PE3)
* 数据网络: OV_Y2(PC6), OV_Y3(PC7), OV_Y4(PE0), OV_Y5(PE1), OV_Y6(PE4), OV_Y7(PD3), OV_Y8(PE5), OV_Y9(PE6)


* **协控制器 (U8: STM32G030F6P6)**: 与主控相连的 IO 映射网络（包含 PA0~PA15, PB0~PB9, PC14/PC15）