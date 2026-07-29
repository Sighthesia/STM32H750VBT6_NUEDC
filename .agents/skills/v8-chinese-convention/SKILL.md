---
name: v8-chinese-convention
description: 将 V8 匿名上位机协议参数/命令标识符和电机控制命令中文化。包含 19 个参数宏、12 条命令描述字符串、PID/闭环枚举常量、电机控制函数/变量的命名转换规范。用于 MSPM0G3507 双电机驱动工程（main.c / motor_menu_iface.h / motor_menu.c / motor_menu.h）。
---

# V8 中文命名规范

## 转换总则

- **宏定义**：`V8_参数_<中文名>`（协议数值 ID 不变，只改宏标识符）
- **命令字符串**：`names[12]`/`infos[12]` → 中文描述文本（发给上位机）
- **函数/变量**：`<领域>_<动作>_<对象>`（如 `闭环_启动`、`菜单_应用电机`）
- **保留**：通用技术缩写如 `PID`、`UART`、`GPIO`、`I2C`、`SPI`、`EXTI`、`PWM` 不翻译
- **同步**：`motor_menu_iface.h` 的 `extern` 声明与 `main.c` 定义保持一致

## 1. V8 协议参数宏

19 个参数 ID 宏在 `main.c` 顶部。只改宏名，不改数值。

| 旧宏名 | 新宏名 |
|--------|--------|
| `V8_PARAM_M1_SPEED` | `V8_参数_M1_速度` |
| `V8_PARAM_M1_DIRECTION` | `V8_参数_M1_方向` |
| `V8_PARAM_M2_SPEED` | `V8_参数_M2_速度` |
| `V8_PARAM_M2_DIRECTION` | `V8_参数_M2_方向` |
| `V8_PARAM_BASE_RPM` | `V8_参数_基准转速` |
| `V8_PARAM_YAW_TARGET` | `V8_参数_偏航目标` |
| `V8_PARAM_YAW_RATE` | `V8_参数_偏航速率` |
| `V8_PARAM_PID_L_KP` | `V8_参数_PID左_KP` |
| `V8_PARAM_PID_L_KI` | `V8_参数_PID左_KI` |
| `V8_PARAM_PID_L_KD` | `V8_参数_PID左_KD` |
| `V8_PARAM_PID_R_KP` | `V8_参数_PID右_KP` |
| `V8_PARAM_PID_R_KI` | `V8_参数_PID右_KI` |
| `V8_PARAM_PID_R_KD` | `V8_参数_PID右_KD` |
| `V8_PARAM_PID_RATE_KP` | `V8_参数_PID角速率_KP` |
| `V8_PARAM_PID_RATE_KI` | `V8_参数_PID角速率_KI` |
| `V8_PARAM_PID_RATE_KD` | `V8_参数_PID角速率_KD` |
| `V8_PARAM_PID_YAW_KP` | `V8_参数_PID偏航_KP` |
| `V8_PARAM_PID_YAW_KI` | `V8_参数_PID偏航_KI` |
| `V8_PARAM_PID_YAW_KD` | `V8_参数_PID偏航_KD` |
| `V8_PARAMETER_COUNT` | `V8_参数_数量` |

替换步骤：
1. 编辑定义块替换全部 20 行
2. 对每个旧宏名使用 `edit replaceAll` 更新所有引用

## 2. 上位机页面宏

| 旧名 | 新名 |
|------|------|
| `ASSISTANT_PAGE_LEFT` | `上位机页面_左轮` |
| `ASSISTANT_PAGE_RIGHT` | `上位机页面_右轮` |
| `ASSISTANT_PAGE_RATE` | `上位机页面_角速率` |
| `ASSISTANT_PAGE_YAW` | `上位机页面_偏航` |
| `ASSISTANT_PAGE_COUNT` | `上位机页面_数量` |

## 3. 命令描述字符串

`v8_发送命令信息()` 函数中的两个字符串数组：

```c
static const char *const names[12] = {
    "调试_进入", "调试_退出", "电机_急停",
    "M1_启动", "M2_启动", "M1_停止", "M2_停止",
    "闭环_使能", "闭环_禁止", "闭环_角速率", "闭环_偏航", "闭环_停止"
};
static const char *const infos[12] = {
    "进入电机调试模式", "退出电机调试模式", "停止双电机",
    "启动电机 1", "启动电机 2", "停止电机 1", "停止电机 2",
    "使能 PID 闭环", "禁止 PID 闭环",
    "直接偏航角速率控制", "偏航保持模式", "停止 PID 闭环"
};
```

## 4. PID 配置常量

| 旧名 | 新名 |
|------|------|
| `PID_CONTROL_PERIOD_S` | `PID_控制周期_秒` |
| `PID_MOTOR_OUTPUT_LIMIT` | `PID_电机输出限制` |
| `PID_WHEEL_RPM_LIMIT` | `PID_轮速限制` |
| `PID_YAW_RATE_LIMIT` | `PID_偏航速率限制` |
| `PID_YAW_DIFF_RPM_LIMIT` | `PID_偏航差速限制` |

## 5. 闭环模式枚举

```c
typedef enum {
    闭环_角速率模式 = 0u,
    闭环_偏航模式   = 1u,
} 闭环_模式枚举;
```

## 6. 电机控制函数/变量

### 菜单接口函数（在 main.c 定义 + motor_menu_iface.h 声明）

| 旧名 | 新名 |
|------|------|
| `motor_menu_apply` | `菜单_应用电机` |
| `motor_menu_toggle_dir` | `菜单_切换方向` |
| `motor_menu_set_duty` | `菜单_设置占空比` |
| `motor_menu_init` | `菜单_初始化` |
| `motor_menu_update_display` | `菜单_更新显示` |

### 共享状态变量（main.c 定义 + motor_menu_iface.h extern 声明）

| 旧名 | 新名 |
|------|------|
| `g_motor_enable` | `g_电机使能` |
| `g_motor_reverse` | `g_电机方向` |
| `g_motor_count` | `g_电机计数` |
| `g_motor_rpm` | `g_电机转速` |
| `g_menu_duty` | `g_菜单占空比` |
| `g_comm_active` | `g_通讯活跃` |
| `g_drop_count` | `g_丢弃计数` |
| `MOTOR_MENU_DUTY_MAX` | `菜单_占空比上限` |

### 内部函数/变量（main.c static）

| 旧名 | 新名 |
|------|------|
| `motor_state_t` | `电机状态_t` |
| `apply_motor_internal` | `应用电机内部` |
| `read_encoder` | `读取编码器` |
| `clamp_float` | `夹紧浮点` |
| `abs_duty` | `占空比绝对值` |
| `motor2_enc_raw` | `电机2_编码器原始值` |

### 闭环变量/函数

| 旧名 | 新名 |
|------|------|
| `closed_loop_enabled` | `闭环_使能` |
| `closed_loop_mode` | `闭环_模式` |
| `closed_loop_base_rpm` | `闭环_基准转速` |
| `closed_loop_yaw_target` | `闭环_偏航目标` |
| `closed_loop_yaw_rate_target` | `闭环_偏航速率目标` |
| `closed_loop_yaw_angle` | `闭环_偏航角度` |
| `closed_loop_yaw_rate` | `闭环_偏航角速率` |
| `closed_loop_stop` | `闭环_停止` |
| `closed_loop_start` | `闭环_启动` |
| `closed_loop_update` | `闭环_更新` |

### 上位机协议变量/函数

| 旧名 | 新名 |
|------|------|
| `v8_debug_mode` | `v8_调试模式` |
| `v8_debug_speed` | `v8_调试速度` |
| `v8_debug_direction` | `v8_调试方向` |
| `v8_rx_frame` | `v8_接收缓冲` |
| `v8_rx_length` | `v8_接收长度` |
| `v8_rx_expected_length` | `v8_预期长度` |
| `v8_ms_tick` | `v8_毫秒计时` |
| `v8_last_command_tick` | `v8_上次命令计时` |
| `v8_get_u16` | `v8_读取_u16` |
| `v8_put_u16/put_i16/put_i32` | `v8_写入_u16/写入_i16/写入_i32` |
| `v8_float_to_i16` | `v8_浮点转_i16` |
| `v8_send_frame` | `v8_发送帧` |
| `v8_send_ack` | `v8_发送确认` |
| `v8_send_parameter_info/value/count` | `v8_发送参数信息/值/数量` |
| `v8_send_device_info/command_info` | `v8_发送设备信息/命令信息` |
| `v8_handle_command` | `v8_处理命令` |
| `v8_handle_command_query` | `v8_处理命令查询` |
| `v8_handle_parameter_command` | `v8_处理参数命令` |
| `v8_handle_frame` | `v8_处理帧` |
| `v8_stop_debug_motors` | `v8_停止调试电机` |
| `v8_apply_debug_parameter` | `v8_应用调试参数` |
| `v8_set_parameter` | `v8_设置参数` |
| `v8_poll_rx` | `v8_轮询接收` |
| `v8_report` | `v8_上报遥测` |
| `assistant_page` | `上位机页面` |
| `assistant_param` | `上位机参数` |
| `assistant_apply_parameters` | `上位机_应用参数` |

## 7. 多文件同步注意事项

修改需要同步的文件：

1. `project/user/src/main.c` — 定义和主调用
2. `project/code/motor_menu_iface.h` — extern 声明 + 宏定义
3. `project/code/motor_menu.c` — 菜单回调调用（屏幕禁用时死代码，但须保持一致性）
4. `project/code/motor_menu.h` — 函数声明

安全替换顺序规则：
- 长字符串优先于短字符串（如 `closed_loop_yaw_rate_target` 先于 `closed_loop_yaw_rate`）
- `v8_handle_parameter_command` 先于 `v8_handle_command`
- `closed_loop_mode_enum` 先于 `closed_loop_mode`
- 使用 `edit replaceAll` 确保一次替换所有引用，不漏
