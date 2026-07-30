#include "ZDT_Stepper.h"

/* ---------------------------------- 电机配置表 --------------------------------- */
static zdt_stepper_motor_config_t zdt_stepper_motor_config_table[] = {
    // {串口句柄, DMA句柄, 电机ID}
    {&huart3, NULL, STEPPER_PITCH}};

// 串口管理数组
static ZDTStepperUart uart_motors[ZDT_STEPPER_MAX_UART];
static uint8_t uart_count = 0;

#define ZDT_STEPPER_TX_MAX_LENGTH 13U
#define ZDT_STEPPER_TX_QUEUE_DEPTH 8U

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t active_data[ZDT_STEPPER_TX_MAX_LENGTH];
    uint8_t active_length;
    uint8_t busy;
    uint8_t queue[ZDT_STEPPER_TX_QUEUE_DEPTH][ZDT_STEPPER_TX_MAX_LENGTH];
    uint8_t queue_length[ZDT_STEPPER_TX_QUEUE_DEPTH];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
    uint8_t pending_speed[ZDT_STEPPER_TX_MAX_LENGTH];
    uint8_t pending_speed_length;
    uint8_t speed_pending;
    uint32_t dropped_commands;
    uint32_t transmit_errors;
} ZDTStepperTxState;

static ZDTStepperTxState uart_tx[ZDT_STEPPER_MAX_UART];

static ZDTStepperTxState *ZDT_Stepper_Get_Tx_State(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < uart_count; i++)
    {
        if (uart_tx[i].huart == huart)
        {
            return &uart_tx[i];
        }
    }

    return NULL;
}

static void ZDT_Stepper_Start_Next_Transmit(ZDTStepperTxState *state)
{
    if (!state || state->busy)
    {
        return;
    }

    const uint8_t *data = NULL;
    uint8_t length = 0;
    uint8_t from_queue = 0;
    uint8_t queue_index = 0;

    if (state->queue_count > 0U)
    {
        data = state->queue[state->queue_head];
        length = state->queue_length[state->queue_head];
        from_queue = 1;
        queue_index = state->queue_head;
    }
    else if (state->speed_pending)
    {
        data = state->pending_speed;
        length = state->pending_speed_length;
    }
    else
    {
        return;
    }

    memcpy(state->active_data, data, length);
    state->active_length = length;
    state->busy = 1U;
    if (from_queue)
    {
        state->queue_head = (uint8_t)((state->queue_head + 1U) % ZDT_STEPPER_TX_QUEUE_DEPTH);
        state->queue_count--;
    }
    else
    {
        state->speed_pending = 0U;
    }

    if (zdt_stepper_hal.Uart_Transmit(state->huart, state->active_data, length) != HAL_OK)
    {
        state->busy = 0U;
        state->transmit_errors++;
        if (from_queue)
        {
            state->queue_head = queue_index;
            state->queue_count++;
        }
        else
        {
            state->speed_pending = 1U;
        }
    }
}

// 全局电机对象指针数组
ZDTStepperMotor *zdt_stepper_motors[ZDT_STEPPER_MOTOR_CONFIG_NUM] = {0};

void print_hex_array(const uint8_t *data, uint16_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        printf("%02X ", data[i]); // 每个字节打印为两位十六进制
    }
    printf("\n");
}

/**
 *****************************************************************************
 * @brief   遍历配置表自动初始化所有步进电机及串口
 *****************************************************************************
 */
void ZDT_Stepper_Init(void)
{
    uart_count = 0;
    memset(uart_motors, 0, sizeof(uart_motors));
    memset(uart_tx, 0, sizeof(uart_tx));

    for (uint32_t i = 0; i < ZDT_STEPPER_MOTOR_CONFIG_NUM; i++)
    {
        UART_HandleTypeDef *huart = zdt_stepper_motor_config_table[i].huart;
        DMA_HandleTypeDef *hdma_rx = zdt_stepper_motor_config_table[i].hdma_rx;
        uint8_t motor_id = zdt_stepper_motor_config_table[i].motor_id;

        int uart_idx = -1;
        for (int j = 0; j < uart_count; j++)
        {
            if (uart_motors[j].huart == huart)
            {
                uart_idx = j;
                break;
            }
        }
        if (uart_idx == -1 && uart_count < ZDT_STEPPER_MAX_UART)
        {
            uart_motors[uart_count].huart = huart;
            uart_motors[uart_count].hdma_rx = hdma_rx;
            uart_motors[uart_count].motor_count = 0;
            uart_tx[uart_count].huart = huart;
            uart_idx = uart_count;
            uart_count++;
        }
        if (uart_idx != -1 && uart_motors[uart_idx].motor_count < ZDT_STEPPER_PER_UART_MAX_MOTORS)
        {
            int m = uart_motors[uart_idx].motor_count;
            uart_motors[uart_idx].motors[m].id = motor_id;
            uart_motors[uart_idx].motors[m].huart = huart;
            uart_motors[uart_idx].motors[m].hdma_rx = hdma_rx;
            memset(uart_motors[uart_idx].motors[m].rx_buffer, 0, sizeof(uart_motors[uart_idx].motors[m].rx_buffer));
            uart_motors[uart_idx].motor_count++;
            // 保存指针
            zdt_stepper_motors[i] = &uart_motors[uart_idx].motors[m];
        }
    }
}

/**
 * @brief 注册电机到指定串口
 * @param huart 串口句柄
 * @param motor_id 电机ID
 */
void ZDT_Stepper_Register_Motor(UART_HandleTypeDef *huart, uint8_t motor_id)
{
    // 查找对应的串口
    for (int i = 0; i < uart_count; i++)
    {
        if (uart_motors[i].huart == huart)
        {
            // 检查该电机是否已注册
            for (int j = 0; j < uart_motors[i].motor_count; j++)
            {
                if (uart_motors[i].motors[j].id == motor_id)
                {
                    return; // 电机已注册
                }
            }

            // 添加新电机
            if (uart_motors[i].motor_count < ZDT_STEPPER_PER_UART_MAX_MOTORS)
            {
                uart_motors[i].motors[uart_motors[i].motor_count].id = motor_id;
                uart_motors[i].motors[uart_motors[i].motor_count].huart = huart;
                uart_motors[i].motors[uart_motors[i].motor_count].hdma_rx = uart_motors[i].hdma_rx;
                memset(uart_motors[i].motors[uart_motors[i].motor_count].rx_buffer, 0, sizeof(uart_motors[i].motors[uart_motors[i].motor_count].rx_buffer));
                uart_motors[i].motor_count++;
            }
            return;
        }
    }
}

/**
 * @brief 获取电机对象
 * @param huart 串口句柄
 * @param motor_id 电机ID
 * @return 电机对象指针，未找到返回NULL
 */
ZDTStepperMotor *ZDT_Stepper_Get_Motor(UART_HandleTypeDef *huart, uint8_t motor_id)
{
    // 查找对应的串口
    for (int i = 0; i < uart_count; i++)
    {
        if (uart_motors[i].huart == huart)
        {
            // 查找电机
            for (int j = 0; j < uart_motors[i].motor_count; j++)
            {
                if (uart_motors[i].motors[j].id == motor_id)
                {
                    return &uart_motors[i].motors[j];
                }
            }
            return NULL;
        }
    }
    return NULL;
}

// 根据指令标识赋值的函数
void assign_message_based_on_command(uint8_t command_id, char *message, size_t message_size)
{
    // 清空字符串
    memset(message, 0, message_size);

    // 根据指令标识赋值
    switch (command_id)
    {
    case 0xF3:
        snprintf(message, message_size, "电机使能");
        break;
    case 0xF5:
        snprintf(message, message_size, "力矩控制");
        break;
    case 0xF6:
        snprintf(message, message_size, "速度控制");
        break;
    case 0xFB:
        snprintf(message, message_size, "直通位置控制");
        break;
    case 0xFD:
        snprintf(message, message_size, "梯形位置控制");
        break;
    case 0xFE:
        snprintf(message, message_size, "立即停止");
        break;
    case 0xFF:
        snprintf(message, message_size, "多机同步运动");
        break;
    case 0x0A:
        snprintf(message, message_size, "角度清零");
        break;
    case 0x0E:
        snprintf(message, message_size, "解除堵转保护");
        break;
    case 0x06:
        snprintf(message, message_size, "触发编码器校准");
        break;
    case 0x84:
        snprintf(message, message_size, "修改细分");
        break;
    default:
        snprintf(message, message_size, "未知的指令");
        break;
    }
}

void receive_motor_status(ZDTStepperMotor *motor, uint8_t *data, uint16_t size)
{
    char message[50];       // 字符串缓冲区
    char statu_message[13]; // 状态字符串缓冲区
    if (data == NULL || motor == NULL)
        return; // 检查输入是否为空
    if (size == 4 && data[1] != 0x3A)
    {
        uint8_t motor_id = data[0];   // 电机地址
        uint8_t command_id = data[1]; // 指令标识
        uint8_t status = data[2];     // 执行状态
        uint8_t checksum = data[3];   // 校验字节
        if (checksum != 0x6B)
        {
            printf("校验失败!\n");
            return; // 校验失败，返回
        }
        snprintf(statu_message, sizeof(statu_message), (status == 0x02) ? "执行成功" : "执行失败");
        assign_message_based_on_command(command_id, message, sizeof(message));
        printf("电机 %d:%s:%s\n", motor_id, message, statu_message);
        // 这里可以根据需要进一步处理接收到的指令
    }
    else
    {
        uint8_t motor_id = data[0];        // 电机地址
        uint8_t command_id = data[1];      // 指令标识
        uint8_t checksum = data[size - 1]; // 校验字节
        uint32_t temp;
        if (checksum != 0x6B)
        {
            printf("校验失败!\n");
            return; // 校验失败，返回
        }

        // 清空消息
        snprintf(message, sizeof(message), "电机 %d: ", motor_id);

        switch (command_id)
        {
        case 0x1F:
            motor->data.firmware_version = (data[2] << 8) | data[3]; // 固件版本
            motor->data.hardware_version = (data[4] << 8) | data[5]; // 硬件版本
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "固件版本: %d, 硬件版本: %d", motor->data.firmware_version, motor->data.hardware_version);
            break;

        case 0x20:
            motor->data.phase_resistance = (data[2] << 8) | data[3]; // 相电阻
            motor->data.phase_inductance = (data[4] << 8) | data[5]; // 相电感
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "相电阻: %d mΩ, 相电感: %d µH", motor->data.phase_resistance, motor->data.phase_inductance);
            break;

        case 0x24:
            motor->data.bus_voltage = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "总线电压: %d mV", motor->data.bus_voltage);
            break;

        case 0x26:
            motor->data.bus_average_current = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "总线平均电流: %d mA", motor->data.bus_average_current);
            break;

        case 0x27:
            motor->data.phase_current = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "相电流: %d mA", motor->data.phase_current);
            break;

        case 0x29:
            motor->data.encoder_raw_value = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "编码器原始值: %d", motor->data.encoder_raw_value);
            break;

        case 0x31:
            motor->data.encoder_calibrated_value = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "经过线性化校准后的编码器值: %d", motor->data.encoder_calibrated_value);
            break;

        case 0x33:
            temp = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]; // 组合四个字节
            motor->data.target_position = (data[2] == 0x01) ? -((float)temp) / 10.0f : ((float)temp) / 10.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "目标位置: %.1f °", motor->data.target_position);
            break;

        case 0x35:
            temp = (data[3] << 8) | data[4]; // 组合两个字节
            motor->data.current_speed = (data[2] == 0x01) ? -((float)temp) / 10.0f : ((float)temp) / 10.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "实时转速: %.1f RPM", motor->data.current_speed);
            break;

        case 0x36:
            temp = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]; // 组合四个字节
            motor->data.current_position = (data[2] == 0x01) ? -((float)temp) / 10.0f : ((float)temp) / 10.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "实时位置: %.1f °", motor->data.current_position);
            break;

        case 0x37:
            temp = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]; // 组合四个字节
            motor->data.position_error = (data[2] == 0x01) ? -((float)temp) / 10.0f : ((float)temp) / 10.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "位置误差: %.1f °", motor->data.position_error);
            break;

        case 0x39:
            motor->data.driver_temperature = (data[2] == 0x01) ? -((int8_t)data[3]) : ((int8_t)data[3]);
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "驱动器实时温度: %.d °C", motor->data.driver_temperature);
            break;

        case 0x3A:
            motor->data.motor_status_flags = data[2];
            motor->data.motor_enabled = (data[2] & 0x01);
            motor->data.motor_position_reached = (data[2] & 0x02) >> 1;
            motor->data.motor_stall = (data[2] & 0x04) >> 2;
            motor->data.motor_stall_protection = (data[2] & 0x08) >> 3;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "status=0x%02X", motor->data.motor_status_flags);
            break;

        default:
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "未知命令");
            break;
        }

        // 最后统一打印
        printf("%s\n", message);
    }
}

void ZDT_Stepper_USRT_RX_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 查找对应的串口和电机
    for (int i = 0; i < uart_count; i++)
    {
        if (uart_motors[i].huart == huart)
        {
            // 遍历该串口下的所有电机，查找数据来源
            for (int j = 0; j < uart_motors[i].motor_count; j++)
            {
                ZDTStepperMotor *motor = &uart_motors[i].motors[j];
                if (motor->rx_buffer[0] == motor->id)
                {
                    // 找到对应的电机
                    receive_motor_status(motor, motor->rx_buffer, Size);
                    // 注意：这里不再切换GPIO状态，由用户在应用层自行处理
                    break;
                }
            }
            // 重新启动DMA接收
            if (uart_motors[i].motor_count > 0)
            {
                HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_motors[i].motors[0].rx_buffer, sizeof(uart_motors[i].motors[0].rx_buffer));
                // 移除特定DMA中断的禁用，由用户在应用层自行处理
            }
            return;
        }
    }
}

/**
 *****************************************************************************
 * @brief   通过HAL抽象层发送数据到步进电机
 * @param   motor 电机对象
 * @param   data  数据指针
 * @param   length 数据长度
 *****************************************************************************
 */
void ZDT_Stepper_UART_Send_Data(ZDTStepperMotor *motor, uint8_t *data, size_t length)
{
    if (!motor || !motor->huart || !data || length == 0U || length > ZDT_STEPPER_TX_MAX_LENGTH)
    {
        return;
    }

    ZDTStepperTxState *state = ZDT_Stepper_Get_Tx_State(motor->huart);
    if (!state)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (data[1] == 0xF6U)
    {
        /* PID speed updates supersede older pending speed requests on this UART. */
        memcpy(state->pending_speed, data, length);
        state->pending_speed_length = (uint8_t)length;
        state->speed_pending = 1U;
    }
    else if (state->queue_count < ZDT_STEPPER_TX_QUEUE_DEPTH)
    {
        memcpy(state->queue[state->queue_tail], data, length);
        state->queue_length[state->queue_tail] = (uint8_t)length;
        state->queue_tail = (uint8_t)((state->queue_tail + 1U) % ZDT_STEPPER_TX_QUEUE_DEPTH);
        state->queue_count++;

        if (data[1] == 0xFEU)
        {
            /* A stop must not be followed by an obsolete speed request. */
            state->speed_pending = 0U;
        }
    }
    else
    {
        state->dropped_commands++;
    }

    ZDT_Stepper_Start_Next_Transmit(state);
    __set_PRIMASK(primask);
}

void ZDT_Stepper_UART_TxCplt_Callback(UART_HandleTypeDef *huart)
{
    ZDTStepperTxState *state = ZDT_Stepper_Get_Tx_State(huart);
    if (!state)
    {
        return;
    }

    state->busy = 0U;
    ZDT_Stepper_Start_Next_Transmit(state);
}

void ZDT_Stepper_UART_Error_Callback(UART_HandleTypeDef *huart)
{
    ZDTStepperTxState *state = ZDT_Stepper_Get_Tx_State(huart);
    if (!state || huart->gState == HAL_UART_STATE_BUSY_TX)
    {
        return;
    }

    state->busy = 0U;
    state->transmit_errors++;
    ZDT_Stepper_Start_Next_Transmit(state);
}
/* 自定义部分结束 */

void ZDT_Stepper_Enable(ZDTStepperMotor *motor, uint8_t enable, uint8_t sync_flag)
{
    if (!motor)
        return;

    uint8_t command[6];
    command[0] = motor->id; // 地址
    command[1] = 0xF3;      // 命令头
    command[2] = 0xAB;      // 命令码
    command[3] = enable;    // 使能状态 (1: 使能, 0: 不使能)
    command[4] = sync_flag; // 多机同步标志位 (0: 不同步, 1: 同步)
    command[5] = 0x6B;      // 校验字节

    ZDT_Stepper_UART_Send_Data(motor, command, 6);
}

/** ***************************************************************************
 * @brief 设置步进电机速度
 *
 * @param motor 电机对象
 * @param dir 方向 CW 反转 顺时针 CCW 正转 逆时针
 * @param speed_f 速度 单位 RPM
 * @param accel_accel 加速度档位（0-255，0表示不使用曲线加减速）
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 ******************************************************************************/
void ZDT_Stepper_Set_Speed(ZDTStepperMotor *motor, uint8_t dir, float speed_f, uint8_t accel_accel, uint8_t sync_flag)
{
    if (!motor)
        return;

    uint16_t speed = (uint16_t)(speed_f * 10); // 转换为两个字节的数据
    // 命令格式：地址 + 0xF6 + 方向 + 速度高 + 速度低 + 加速度档位 + 多机同步标志 + 校验字节
    uint8_t command[8];
    command[0] = motor->id;           // 地址
    command[1] = 0xF6;                // 命令头
    command[2] = dir;                 // 方向 (01: CCW, 00: CW)
    command[3] = (speed >> 8) & 0xFF; // 速度高位
    command[4] = speed & 0xFF;        // 速度低位
    command[5] = accel_accel;         // 加速度档位（1字节）
    command[6] = sync_flag;           // 多机同步标志位
    command[7] = 0x6B;                // 校验字节

    ZDT_Stepper_UART_Send_Data(motor, command, 8);
}

/**
 * @brief 设置步进电机位置
 *
 * @param motor 电机对象
 * @param dir 方向 CW 反转 顺时针 CCW 正转 逆时针
 * @param speed_f 速度  单位 RPM
 * @param position_angle_f 位置角度 单位 度
 * @param position_mode 模式 REL_POS_MODE 相对位置模式 ABS_POS_MODE 绝对位置模式
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_Set_position(ZDTStepperMotor *motor, uint8_t dir, float speed_f, float position_angle_f, uint8_t position_mode, uint8_t sync_flag)
{
    if (!motor)
        return;

    // 1.8°/step, 16细分, 一圈脉冲数 = 360/1.8*16 = 3200
    // 角度转脉冲数: pulse = angle / 360 * 3200
    float pulses_per_circle = 360.0f / 1.8f * 16.0f;                                           // 3200
    uint32_t pulse_count = (uint32_t)((position_angle_f / 360.0f) * pulses_per_circle + 0.5f); // 四舍五入

    uint16_t speed = (uint16_t)(speed_f * 10); // 速度乘以 10
    uint8_t accel_accel = 0;                   // 默认加速度档位为0（直线加减速）

    // 命令格式：地址 + 0xFD + 方向 + 速度高 + 速度低 + 加速度档位 + 脉冲数(4字节) + 相对/绝对 + 同步 + 校验
    uint8_t command[13];
    command[0] = motor->id;                  // 地址
    command[1] = 0xFD;                       // 命令头
    command[2] = dir;                        // 方向 (01: CCW, 00: CW)
    command[3] = (speed >> 8) & 0xFF;        // 速度高位
    command[4] = speed & 0xFF;               // 速度低位
    command[5] = accel_accel;                // 加速度档位（1字节）
    command[6] = (pulse_count >> 24) & 0xFF; // 脉冲数高位
    command[7] = (pulse_count >> 16) & 0xFF; // 脉冲数次高位
    command[8] = (pulse_count >> 8) & 0xFF;  // 脉冲数次低位
    command[9] = pulse_count & 0xFF;         // 脉冲数低位
    command[10] = position_mode;             // 相对/绝对位置标志
    command[11] = sync_flag;                 // 多机同步标志位
    command[12] = 0x6B;                      // 校验字节

    ZDT_Stepper_UART_Send_Data(motor, command, 13);
}

/** ***************************************************************************
 * @brief 设置步进电机位置模式（脉冲数）
 *
 * @param motor 电机对象
 * @param dir 方向 CW 反转 顺时针 CCW 正转 逆时针
 * @param speed_f 速度 单位 RPM
 * @param accel_accel 加速度档位（0-255，0表示不使用曲线加减速）
 * @param pulse_count 脉冲数（uint32_t，单位：脉冲）
 * @param position_mode 位置模式 REL_POS_MODE 相对位置模式 ABS_POS_MODE 绝对位置模式
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 ******************************************************************************/
void ZDT_Stepper_Set_T_position(ZDTStepperMotor *motor, uint8_t dir, float speed_f, uint8_t accel_accel, uint32_t pulse_count, uint8_t position_mode, uint8_t sync_flag)
{
    if (!motor)
        return;

    uint16_t speed = (uint16_t)(speed_f * 10); // 速度乘以 10

    // 命令格式：地址 + 0xFD + 方向 + 速度高 + 速度低 + 加速度档位 + 脉冲数(4字节) + 相对/绝对 + 同步 + 校验
    uint8_t command[13];
    command[0] = motor->id;                  // 地址
    command[1] = 0xFD;                       // 命令头
    command[2] = dir;                        // 方向 (01: CCW, 00: CW)
    command[3] = (speed >> 8) & 0xFF;        // 速度高位
    command[4] = speed & 0xFF;               // 速度低位
    command[5] = accel_accel;                // 加速度档位（1字节）
    command[6] = (pulse_count >> 24) & 0xFF; // 脉冲数高位
    command[7] = (pulse_count >> 16) & 0xFF; // 脉冲数次高位
    command[8] = (pulse_count >> 8) & 0xFF;  // 脉冲数次低位
    command[9] = pulse_count & 0xFF;         // 脉冲数低位
    command[10] = position_mode;             // 相对/绝对位置标志
    command[11] = sync_flag;                 // 多机同步标志位
    command[12] = 0x6B;                      // 校验字节

    ZDT_Stepper_UART_Send_Data(motor, command, 13);
}

/**
 * @brief 力矩控制模式
 *
 * @param motor 电机对象
 * @param dir 方向 CW 反转 顺时针 CCW 正转 逆时针
 * @param current_rate 电流加速度 单位 mA/s
 * @param current 目标电流 单位 mA
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_torque_control(ZDTStepperMotor *motor, uint8_t dir, uint16_t current_rate, uint16_t current, uint8_t sync_flag)
{
    if (!motor)
        return;

    // 命令格式
    uint8_t command[9];
    command[0] = motor->id;                  // 地址
    command[1] = 0xF5;                       // 命令头
    command[2] = dir;                        // 符号 (00: 正电流, 01: 负电流)
    command[3] = (current_rate >> 8) & 0xFF; // 电流斜率高位
    command[4] = current_rate & 0xFF;        // 电流斜率低位
    command[5] = (current >> 8) & 0xFF;      // 电流高位
    command[6] = current & 0xFF;             // 电流低位
    command[7] = sync_flag;                  // 多机同步标志位
    command[8] = 0x6B;                       // 校验字节

    ZDT_Stepper_UART_Send_Data(motor, command, 9);
}

/**
 * @brief 开启多机同步运动
 *
 * @param motor 电机对象
 */
void ZDT_Stepper_start_sync_motion(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    // 命令格式
    uint8_t command[4];
    command[0] = motor->id; // 地址
    command[1] = 0xFF;      // 命令头
    command[2] = 0x66;      // 多机同步运动命令
    command[3] = 0x6B;      // 校验字节

    ZDT_Stepper_UART_Send_Data(motor, command, 4);
}

/**
 * @brief 立即停止
 *
 * @param motor 电机对象
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_stop(ZDTStepperMotor *motor, uint8_t sync_flag)
{
    if (!motor)
        return;

    // 命令格式
    uint8_t command[5];
    command[0] = motor->id; // 地址
    command[1] = 0xFE;      // 命令头
    command[2] = 0x98;      // 立即停止命令
    command[3] = sync_flag; // 多机同步标志位
    command[4] = 0x6B;      // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 5);
}

/**
 * @brief 角度清零
 *
 * @param motor 电机对象
 */
void ZDT_Stepper_clear_current_position(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    // 命令格式
    uint8_t command[4];
    command[0] = motor->id; // 地址
    command[1] = 0x0A;      // 命令头
    command[2] = 0x6D;      // 固定字节
    command[3] = 0x6B;      // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 4);
}

/**
 * @brief 触发回零
 *
 * @param motor 电机对象
 * @param return_mode 回零模式
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_trigger_zero_return(ZDTStepperMotor *motor, uint8_t return_mode, uint8_t sync_flag)
{
    if (!motor)
        return;

    // 命令格式
    uint8_t command[5];
    command[0] = motor->id;   // 地址
    command[1] = 0x9A;        // 命令头
    command[2] = return_mode; // 回零模式
    command[3] = sync_flag;   // 多机同步标志
    command[4] = 0x6B;        // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 5);
}

/**
 * @brief 解除堵转保护
 *
 * @param motor 电机对象
 */
void ZDT_Stepper_release_stall_protection(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    // 命令格式
    uint8_t command[4];
    command[0] = motor->id; // 地址
    command[1] = 0x0E;      // 命令头
    command[2] = 0x52;      // 固定字节
    command[3] = 0x6B;      // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 4);
}

/**
 * @brief 触发编码器校准
 *
 * @param motor 电机对象
 */
void ZDT_Stepper_trigger_encoder_calibration(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    // 命令格式
    uint8_t command[4];
    command[0] = motor->id; // 地址
    command[1] = 0x06;      // 命令头
    command[2] = 0x45;      // 固定字节
    command[3] = 0x6B;      // 校验字节

    ZDT_Stepper_UART_Send_Data(motor, command, 4);
}

void ZDT_Stepper_set_microstepping(ZDTStepperMotor *motor, uint8_t store_flag, uint8_t microstepping_value)
{
    if (!motor)
        return;

    // 命令格式
    uint8_t command[6];
    command[0] = motor->id;           // 电机地址
    command[1] = 0x84;                // 命令标识
    command[2] = 0x8A;                // 子命令标识
    command[3] = store_flag;          // 是否存储标志
    command[4] = microstepping_value; // 细分值
    command[5] = 0x6B;                // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 6);
}

/**
 * @brief 读取固件和硬件版本
 *
 * @param motor 电机对象
 */
void ZDT_Stepper_Read_version(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    // 命令格式
    uint8_t command[3];
    command[0] = motor->id; // 地址
    command[1] = 0x1F;      // 命令头
    command[2] = 0x6B;      // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

/**
 * @brief 读取相电阻和相电感
 *
 * @param motor 电机对象
 */
void ZDT_Stepper_Read_resistance_and_inductance(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x20;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

void ZDT_Stepper_Read_bus_voltage(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x24;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

void ZDT_Stepper_Read_bus_average_current(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x26;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

void ZDT_Stepper_Read_phase_current(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x27;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

/**
 * @brief 读取编码器原始值
 * 编码器原始值一圈的数值是 0-16383，即 0-16383 表示 0-360°
 * @param motor 电机对象
 */
void ZDT_Stepper_Read_encoder_raw_value(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x29;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

/**
 * @brief 经过线性化校准后的编码器值
 * 经过线性化校准后，内部对编码器值进行了 4 倍频，一圈的数值是 0-65535
 * @param motor 电机对象
 */
void ZDT_Stepper_Read_encoder_calibrated_value(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x31;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

void ZDT_Stepper_Read_target_position(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x33;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

void ZDT_Stepper_Read_current_speed(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x35;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

void ZDT_Stepper_Read_current_position(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x36;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

void ZDT_Stepper_Read_position_error(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x37;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

void ZDT_Stepper_Read_driver_temperature(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x39;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}

void ZDT_Stepper_Read_motor_status_flags(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = motor->id;    // 电机地址
    command[1] = 0x3A;         // 命令头
    command[2] = 0x6B;         // 校验字节
    ZDT_Stepper_UART_Send_Data(motor, command, 3);
}
