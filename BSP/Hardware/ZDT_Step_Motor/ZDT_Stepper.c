#include "ZDT_Stepper.h"

/* ---------------------------------- 电机配置表 --------------------------------- */
static zdt_stepper_motor_config_t zdt_stepper_motor_config_table[] = {
    {&hfdcan2, STEPPER_PITCH}};

static ZDTStepperCan can_motors;

#define ZDT_STEPPER_TX_MAX_LENGTH 13U
#define ZDT_STEPPER_TX_QUEUE_DEPTH 8U
#define ZDT_STEPPER_COMMAND_ACK_TIMEOUT_MS 100U

static bool ZDT_Stepper_Is_Control_Command(uint8_t command)
{
    switch (command)
    {
    case 0xF3U: case 0xF5U: case 0xF6U: case 0xFBU: case 0xFDU:
    case 0xFEU: case 0xFFU: case 0x0AU: case 0x0EU: case 0x06U:
    case 0x84U: case 0x9AU: case 0x9CU:
        return true;
    default:
        return false;
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
bool ZDT_Stepper_Init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    memset(&can_motors, 0, sizeof(can_motors));
    memset(zdt_stepper_motors, 0, sizeof(zdt_stepper_motors));
    can_motors.hfdcan = zdt_stepper_motor_config_table[0].hfdcan;
    can_motors.motors[0].id = zdt_stepper_motor_config_table[0].motor_id;
    can_motors.motors[0].hfdcan = can_motors.hfdcan;
    can_motors.motors[0].data.microstepping = 16U;
    can_motors.motor_count = 1U;
    zdt_stepper_motors[0] = &can_motors.motors[0];

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = ((uint32_t)can_motors.motors[0].id << 8U);
    filter.FilterID2 = filter.FilterID1 | 0xFFU;
    if (HAL_FDCAN_ConfigFilter(can_motors.hfdcan, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(can_motors.hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(can_motors.hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK ||
        HAL_FDCAN_Start(can_motors.hfdcan) != HAL_OK)
    {
        memset(zdt_stepper_motors, 0, sizeof(zdt_stepper_motors));
        can_motors.motor_count = 0U;
        return false;
    }
    return true;
}

/**
 * @brief 注册电机到指定串口
 * @param huart 串口句柄
 * @param motor_id 电机ID
 */
void ZDT_Stepper_Register_Motor(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    (void)hfdcan;
    (void)motor_id;
}

/**
 * @brief 获取电机对象
 * @param hfdcan FDCAN句柄
 * @param motor_id 电机ID
 * @return 电机对象指针，未找到返回NULL
 */
ZDTStepperMotor *ZDT_Stepper_Get_Motor(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    if (hfdcan != can_motors.hfdcan || motor_id != can_motors.motors[0].id) return NULL;
    return &can_motors.motors[0];
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
    case 0x9A:
        snprintf(message, message_size, "触发回零");
        break;
    case 0x9C:
        snprintf(message, message_size, "中断回零");
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
    if (data == NULL || motor == NULL || size < 4U)
    {
        if (motor != NULL) motor->data.rx_invalid_count++;
        return;
    }
    if (data[0] != motor->id || data[size - 1U] != 0x6BU)
    {
        if (data[0] == motor->id) motor->data.rx_invalid_count++;
        else motor->data.rx_unmatched_count++;
        if (motor->data.command_lifecycle != ZDT_STEPPER_COMMAND_CAN_SENT)
        {
            motor->data.command_lifecycle = data[0] == motor->id ?
                ZDT_STEPPER_COMMAND_RX_INVALID : ZDT_STEPPER_COMMAND_RX_UNMATCHED;
        }
        return;
    }
    if (size == 4 && data[1] != 0x3A && data[1] != 0x3B)
    {
        uint8_t motor_id = data[0];   // 电机地址
        uint8_t command_id = data[1]; // 指令标识
        uint8_t status = data[2];     // 执行状态
        if (motor->data.command_lifecycle != ZDT_STEPPER_COMMAND_CAN_SENT ||
            motor->data.pending_command != command_id)
        {
            motor->data.rx_unmatched_count++;
            if (motor->data.command_lifecycle != ZDT_STEPPER_COMMAND_CAN_SENT)
            {
                motor->data.command_lifecycle = ZDT_STEPPER_COMMAND_RX_UNMATCHED;
            }
            return;
        }
        snprintf(statu_message, sizeof(statu_message), (status == 0x02) ? "执行成功" : "执行失败");
        motor->data.last_command = command_id;
        motor->data.last_command_status = status;
        motor->data.last_command_ms = HAL_GetTick();
        motor->data.pending_command = 0U;
        motor->data.command_lifecycle = status == 0x02U ?
            ZDT_STEPPER_COMMAND_ACK_OK : ZDT_STEPPER_COMMAND_ACK_ERROR;
        assign_message_based_on_command(command_id, message, sizeof(message));
        printf("电机 %d:%s:%s\n", motor_id, message, statu_message);
        // 这里可以根据需要进一步处理接收到的指令
    }
    else if (size >= 4U)
    {
        uint8_t motor_id = data[0];        // 电机地址
        uint8_t command_id = data[1];      // 指令标识
        uint32_t temp;

        // 清空消息
        snprintf(message, sizeof(message), "电机 %d: ", motor_id);

        switch (command_id)
        {
        case 0x1F:
            if (size < 7U) break;
            motor->data.firmware_version = (data[2] << 8) | data[3]; // 固件版本
            motor->data.hardware_version = (data[4] << 8) | data[5]; // 硬件版本
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "固件版本: %d, 硬件版本: %d", motor->data.firmware_version, motor->data.hardware_version);
            break;

        case 0x20:
            if (size < 7U) break;
            motor->data.phase_resistance = (data[2] << 8) | data[3]; // 相电阻
            motor->data.phase_inductance = (data[4] << 8) | data[5]; // 相电感
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "相电阻: %d mΩ, 相电感: %d µH", motor->data.phase_resistance, motor->data.phase_inductance);
            break;

        case 0x24:
            if (size < 5U) break;
            motor->data.bus_voltage = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "总线电压: %d mV", motor->data.bus_voltage);
            break;

        case 0x26:
            if (size < 5U) break;
            motor->data.bus_average_current = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "总线平均电流: %d mA", motor->data.bus_average_current);
            break;

        case 0x27:
            if (size < 5U) break;
            motor->data.phase_current = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "相电流: %d mA", motor->data.phase_current);
            break;

        case 0x29:
            if (size < 5U) break;
            motor->data.encoder_raw_value = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "编码器原始值: %d", motor->data.encoder_raw_value);
            break;

        case 0x31:
            if (size < 5U) break;
            motor->data.encoder_calibrated_value = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "经过线性化校准后的编码器值: %d", motor->data.encoder_calibrated_value);
            break;

        case 0x33:
            if (size < 8U) break;
            temp = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 8) | data[6]; // 组合四个字节
            motor->data.target_position = (data[2] == 0x01) ? -((float)temp * 360.0f) / 65536.0f : ((float)temp * 360.0f) / 65536.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "目标位置: %.1f °", motor->data.target_position);
            break;

        case 0x35:
            if (size < 6U) break;
            temp = (data[3] << 8) | data[4]; // 组合两个字节
            motor->data.current_speed = (data[2] == 0x01) ? -((float)temp) / 10.0f : ((float)temp) / 10.0f;
            motor->data.current_speed_ms = HAL_GetTick();
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "实时转速: %.1f RPM", motor->data.current_speed);
            break;

        case 0x36:
            if (size < 8U) break;
            temp = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 8) | data[6]; // 组合四个字节
            motor->data.current_position = (data[2] == 0x01) ? -((float)temp * 360.0f) / 65536.0f : ((float)temp * 360.0f) / 65536.0f;
            motor->data.current_position_ms = HAL_GetTick();
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "实时位置: %.1f °", motor->data.current_position);
            break;

        case 0x37:
            if (size < 8U) break;
            temp = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 8) | data[6]; // 组合四个字节
            motor->data.position_error = (data[2] == 0x01) ? -((float)temp * 360.0f) / 65536.0f : ((float)temp * 360.0f) / 65536.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "位置误差: %.1f °", motor->data.position_error);
            break;

        case 0x39:
            if (size < 5U) break;
            motor->data.driver_temperature = (data[2] == 0x01) ? -((int8_t)data[3]) : ((int8_t)data[3]);
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "驱动器实时温度: %.d °C", motor->data.driver_temperature);
            break;

        case 0x3A:
            if (size < 4U) break;
            motor->data.motor_status_flags = data[2];
            motor->data.motor_enabled = (data[2] & 0x01);
            motor->data.motor_position_reached = (data[2] & 0x02) >> 1;
            motor->data.motor_stall = (data[2] & 0x04) >> 2;
            motor->data.motor_stall_protection = (data[2] & 0x08) >> 3;
            motor->data.motor_status_ms = HAL_GetTick();
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "status=0x%02X", motor->data.motor_status_flags);
            break;

        case 0x3B:
            if (size < 4U) break;
            motor->data.homing_status_flags = data[2];
            motor->data.homing_status_ms = HAL_GetTick();
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "homing=0x%02X", motor->data.homing_status_flags);
            break;

        default:
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "未知命令");
            break;
        }

        if (command_id != 0x35U && command_id != 0x36U &&
            command_id != 0x3AU && command_id != 0x3BU)
        {
            printf("%s\n", message);
        }
    }
}

static uint8_t zdt_can_dlc_bytes(uint32_t dlc)
{
    static const uint8_t sizes[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
    return dlc <= FDCAN_DLC_BYTES_8 ? sizes[dlc] : 0U;
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
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t packet[8];
    size_t offset = 2U;
    uint8_t packet_number = 0U;

    if (!motor || !motor->hfdcan || !data || length < 3U || length > ZDT_STEPPER_TX_MAX_LENGTH)
    {
        return;
    }

    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;

    while (offset < length)
    {
        size_t remaining = length - offset;
        uint8_t payload = remaining > 7U ? 7U : (uint8_t)remaining;
        memset(packet, 0, sizeof(packet));
        packet[0] = data[1];
        memcpy(&packet[1], &data[offset], payload);
        header.Identifier = ((uint32_t)motor->id << 8U) | packet_number++;
        header.DataLength = (uint32_t)((payload + 1U) > 8U ? FDCAN_DLC_BYTES_8 : payload + 1U);
        if (HAL_FDCAN_AddMessageToTxFifoQ(motor->hfdcan, &header, packet) != HAL_OK)
        {
            motor->data.tx_error_count++;
            motor->data.command_lifecycle = ZDT_STEPPER_COMMAND_TIMEOUT;
            return;
        }
        offset += payload;
    }
    if (ZDT_Stepper_Is_Control_Command(data[1]))
    {
        motor->data.pending_command = data[1];
        motor->data.command_lifecycle = ZDT_STEPPER_COMMAND_CAN_SENT;
        motor->data.command_sent_ms = HAL_GetTick();
    }
}

void ZDT_Stepper_FDCAN_RxFifo0_Callback(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (hfdcan != can_motors.hfdcan) return;
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK ||
            header.IdType != FDCAN_EXTENDED_ID ||
            (header.Identifier >> 8U) != can_motors.motors[0].id)
        {
            can_motors.motors[0].data.rx_unmatched_count++;
            continue;
        }
        uint8_t length = zdt_can_dlc_bytes(header.DataLength);
        if (length < 2U || data[0] != can_motors.motors[0].data.pending_command)
        {
            can_motors.motors[0].data.rx_unmatched_count++;
            continue;
        }
        uint8_t response[9] = {0};
        response[0] = can_motors.motors[0].id;
        response[1] = data[0];
        memcpy(&response[2], &data[1], length - 1U);
        receive_motor_status(&can_motors.motors[0], response, (uint16_t)length + 1U);
    }
}

void ZDT_Stepper_FDCAN_Error_Callback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == can_motors.hfdcan) can_motors.motors[0].data.tx_error_count++;
}

void ZDT_Stepper_Update_Communication_Status(ZDTStepperMotor *motor)
{
    if (motor == NULL || motor->data.command_lifecycle != ZDT_STEPPER_COMMAND_CAN_SENT ||
        motor->data.command_sent_ms == 0U)
    {
        return;
    }
    if (HAL_GetTick() - motor->data.command_sent_ms > ZDT_STEPPER_COMMAND_ACK_TIMEOUT_MS)
    {
        motor->data.command_lifecycle = ZDT_STEPPER_COMMAND_TIMEOUT;
        motor->data.command_timeout_count++;
        motor->data.pending_command = 0U;
    }
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

    if (motor->data.limits_enabled != 0U)
    {
        if (motor->data.current_position_ms == 0U ||
            (dir == ZDT_STEPPER_CCW && motor->data.current_position >= motor->data.limit_max) ||
            (dir == ZDT_STEPPER_CW && motor->data.current_position <= motor->data.limit_min))
        {
            ZDT_Stepper_stop(motor, sync_flag);
            return;
        }
    }

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
    ZDT_Stepper_Set_position_accel(motor, dir, speed_f, 0U, position_angle_f, position_mode, sync_flag);
}

void ZDT_Stepper_Set_position_accel(ZDTStepperMotor *motor, uint8_t dir, float speed_f, uint8_t accel, float position_angle_f, uint8_t position_mode, uint8_t sync_flag)
{
    if (!motor)
        return;

    if (motor->data.limits_enabled != 0U && position_mode == ZDT_STEPPER_ABS_POS_MODE)
    {
        float target = dir == ZDT_STEPPER_CCW ? position_angle_f : -position_angle_f;
        if (target < motor->data.limit_min || target > motor->data.limit_max)
            return;
    }

    uint8_t microstepping = motor->data.microstepping == 0U ? 16U : motor->data.microstepping;
    float pulses_per_circle = 360.0f / 1.8f * (float)microstepping;
    uint32_t pulse_count = (uint32_t)((position_angle_f / 360.0f) * pulses_per_circle + 0.5f); // 四舍五入

    uint16_t speed = (uint16_t)(speed_f * 10); // 速度乘以 10
    // 命令格式：地址 + 0xFD + 方向 + 速度高 + 速度低 + 加速度档位 + 脉冲数(4字节) + 相对/绝对 + 同步 + 校验
    uint8_t command[13];
    command[0] = motor->id;                  // 地址
    command[1] = 0xFD;                       // 命令头
    command[2] = dir;                        // 方向 (01: CCW, 00: CW)
    command[3] = (speed >> 8) & 0xFF;        // 速度高位
    command[4] = speed & 0xFF;               // 速度低位
    command[5] = accel;                      // 加速度档位（1字节）
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

    if (motor->data.limits_enabled != 0U && position_mode == ZDT_STEPPER_ABS_POS_MODE)
    {
        uint8_t microstepping = motor->data.microstepping == 0U ? 16U : motor->data.microstepping;
        float angle = (float)pulse_count * 360.0f / (200.0f * (float)microstepping);
        float target = dir == ZDT_STEPPER_CCW ? angle : -angle;
        if (target < motor->data.limit_min || target > motor->data.limit_max)
            return;
    }

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

void ZDT_Stepper_abort_zero_return(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[4] = {motor->id, 0x9C, 0x48, 0x6B};
    ZDT_Stepper_UART_Send_Data(motor, command, sizeof(command));
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
    if (!motor || microstepping_value == 0U)
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
    motor->data.microstepping = microstepping_value;
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

void ZDT_Stepper_Read_homing_status_flags(ZDTStepperMotor *motor)
{
    if (!motor)
        return;

    uint8_t command[3] = {motor->id, 0x3B, 0x6B};
    ZDT_Stepper_UART_Send_Data(motor, command, sizeof(command));
}

void ZDT_Stepper_Set_Software_Limits(ZDTStepperMotor *motor, float minimum, float maximum, bool enabled)
{
    if (!motor)
        return;

    motor->data.limit_min = minimum;
    motor->data.limit_max = maximum;
    motor->data.limits_enabled = enabled && minimum < maximum ? 1U : 0U;
}
