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

## Firmware Branch

- Branch: `car-controller` (`APP_ROLE_CAR` fixed at compile time).
- Worktree: `/home/Sighthesia/0_Files/Producing/Hardware/MCU/STM32/STM32H750VBT6_NUEDC-car-controller`.
- Download artifact: `build/MinSizeRel/STM32H750VBT6_NUEDC.elf`.
- Verify with `git branch --show-current`; do not download this worktree's artifact to the arm controller board.

## Change Boundaries

- Treat `STM32H750VBT6_NUEDC.ioc` and CubeMX-generated sections/files as configuration-owned. Regenerate deliberately and review generated diffs before keeping manual changes.
- Keep application behavior in `Core/Src/main.c` and BSP modules; do not add a second entrypoint.
- `CMakeLists.txt` uses `GLOB_RECURSE ... CONFIGURE_DEPENDS` only for the listed BSP directories, not for every file under `BSP/`.

## 系统

电赛 车载平衡滚球运动控制系统 题目小车。使用两块本工程的STM32H750VBT6开发板作为主控和协控，分别控制小车的运动和闭环步进电机运动控制臂杆管道控制小球保持在指定位置。视觉识别使用 Jetson Nano。

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

车载平衡滚球运动控制系统（H 题）
一、 任务
设计制作一辆载有平衡滚球运动控制装置的循线小车，小车可沿黑色环形路
线行驶，行驶过程中钢球可在滚球运动控制装置内保持相对稳定，如图 1 所示。
车载平衡滚球运动控制系统由小车、带凹槽的摆杆、摆杆控制机构和钢球组成，
如图 2 所示，摆杆左端用铰链或者合页固定，距离小车平板高度 h
≥5cm，另一
端连接至摆杆角度控制机构，可控制钢球稳定在指定位置。
图 1 小车行驶路线示意图
H - 2 / 4
图 2 车载平衡滚球运动控制系统结构参考示意图
二、 要求
1. 安装摆球位置监测图传装置。装置由发送和接收模块组成，图传发送模块
须稳固安装在小车上，接收模块可连接显示存储装置并整体置于环形线路以外，
要求能稳定实时显示钢球在凹槽中滚动的画面，并完整记录每次测试时钢球运动
的视频且能按要求回放。（6 分）
2. 小车置于 A 点，按键启动后沿黑线顺时针行驶一圈并停到 A 点，计时停
止并显示行驶总时间，要求行驶总时间
≤20s，停车偏差
≤2cm。（16 分）
3. 小车在静止状态时，要求摆杆控制装置控制小球从摆杆中心点 O 往+5cm
处运行，到达后折返，再运行到-5cm 处并稳定在该点附近，要求运行时间
≤5s，
±5cm 处的最大误差绝对值
≤1cm。（13 分）
4．小车置于 A 点，钢球置于中心点 O，按键启动后沿黑线顺时针行驶并通
过 B 位置，AB 间行驶时间
≤8s，行驶过程中钢球须稳定在摆杆中心点附近，误
差绝对值
≤1cm。（20 分）
5. 小车置于 A 点，钢球置于中心点 O，按键启动后沿黑线顺时针行驶一圈
并通过 A 位置，整圈行驶总时间
≤30s，要求行驶过程中钢球须稳定在摆杆中心
点附近，误差绝对值
≤1cm。（20 分）
6. 小车置于 A 点，钢球置于摆杆任意指定位置，按键启动后沿黑线顺时针
行驶一圈并通过 A 位置，整圈行驶总时间
≤30s，要求行驶过程中钢球能稳定在
摆杆上的任意指定位置附近，误差绝对值
≤1cm。（20 分）
7. 其他。（5 分）
8. 设计报告。（20 分）
三、 说明
1. 小车运动场地可以参照图 1 用白色广告布喷绘制成，环形线路为黑线，
H - 3 / 4
线宽为 1.8±0.2cm，设定 A、B、C、D 四点， AB、CD 段长度分别为 1.5 米，
BC、DA 段是半径为 0.5 米的半圆弧。整体幅面能满足小车行驶需求，测试中允
许自带。
2. 停车点 A 有长度 5cm、线宽为 1.8±0.2cm、与环路垂直的居中黑色启停
线。启停线沿中心轴线绘制宽 0.1cm、长 30cm 的停车基准虚线，便于判断小车
停车位置。
3. 作品中的车身尺寸长宽不超过 35cm（长）×25cm（宽）。小车采用轮式
驱动，由车载电池供电。为方便测试，需在小车中心轴线上指定测试唯一位置，
整个测试过程中以此判断到达和起始位置偏差。
4. 行驶过程中不得人为干涉、遥控小车运动。行驶过程中小车的投影必须在
轨迹线上，投影完全脱离轨迹线即认为此次测试失败。停车偏差定义为小车上指
定测试位置和基准线（点）间的绝对距离。
5. 小车循迹模块只能使用红外光电模块，数量不限。小车必须有启动按键和
显示装置，按键启动时计时系统开始计时并显示时间，显示屏尺寸不大于 2 英
寸，且须安装在方便观测的位置。
6. 摆球位置监测图传装置的发送模块须稳定安装于车体，摄像头可固定在
摆杆凹槽上方，回传画面能覆盖整个摆杆，能完整清晰看到钢球的滚动运行轨迹
并判定其位置；图传接收模块及其连接的显示存储装置（PAD、笔记本电脑等）
在比赛结束后和作品一起封存。
7. 摆杆结构如图 3 所示，由长度 25cm 的 4 分 PPR 水管（外径 2cm×壁厚
0.34cm）改造而成，槽内放置直径约 1cm 的钢球，改造后整个摆杆的顶视图如图
3（a）所示，摆杆整体要求平直，凹槽内壁表面光滑，保持原状，钢球可灵活在
凹槽中滚动，若内壁有凹坑或做任何增摩擦改造则不予测试；为方便读出钢球运
动坐标位置，凹槽剖面边沿上贴有如图所示的刻度线（不允许贴在凹槽内），刻
度间距 0.1cm；凹槽内钢球位置检测必须采用摄像头。如图 3（b）所示，摆杆截
面外圆弧半径约为 1cm、内圆弧半径约为 0.65cm。整个摆杆不超出车身。
