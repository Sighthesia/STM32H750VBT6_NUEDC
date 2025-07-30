/** ***************************************************************************
 * @File Name: Lazer_Control.c
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-28
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#include "Lazer_Control.h"

uint8_t frame_buffer[FRAME_MAX_LEN];

uint16_t rectangle_position[4][2] = {0}; // 存储矩形四个点的坐标
uint16_t lazer_position[2] = {0};        // 存储激光坐标 (X, Y)

uint8_t rectangle_position_receive_complete = 0;
uint8_t lazer_position_receive_complete = 0;

uint8_t Rectangle_Protocol_Parse(lwrb_t *buff)
{
    uint8_t data;
    uint8_t frame_index = 0;
    uint8_t frame_receiving = 0;
    while (lwrb_read(buff, &data, 1) > 0)
    {
        if (!frame_receiving) // 如果未正在接收帧
        {
            if (data == REC_FRAME_HEAD1)
            {
                frame_receiving = 1;
                frame_index = 0;
                frame_buffer[frame_index] = data;
                frame_index++;
            }
        }
        else if (frame_index == 1) // 如果接收到帧头1
        {
            if (data == REC_FRAME_HEAD2) // 如果接收到帧头2
            {
                frame_buffer[frame_index] = data;
                frame_index++;
            }
            else
            {
                // 帧头2错误，重新等待帧头1
                frame_receiving = 0;
                frame_index = 0;
            }
        }
        else
        {
            frame_buffer[frame_index] = data;
            frame_index++;
            if (frame_index > FRAME_MAX_LEN) // 如果接收到的帧长度超过最大值
            {
                // 超长，丢弃本帧
                frame_receiving = 0;
                frame_index = 0;
            }
            else if (data == FRAME_TAIL && frame_index == FRAME_MAX_LEN) // 如果接收到的帧长度达到最大值, 且为帧尾
            {
                // 帧接收完成，解析数据
                printf("帧接收完成: ");
                for (int i = 0; i < FRAME_MAX_LEN; i++)
                    printf("%02X ", frame_buffer[i]);
                printf("\r\n");

                // 解析坐标
                uint8_t id = frame_buffer[2];
                uint8_t h = frame_buffer[3];
                uint8_t l = frame_buffer[4];
                uint8_t z = frame_buffer[5];

                int value = h + l + z;

                printf("ID: 0x%02X, Value: %d\r\n", id, value);
                // 可根据id区分x/y或角点序号

                switch (id)
                {
                case 0x10: // 第4个点X坐标
                    rectangle_position[3][0] = (value);
                    printf("rectangle_position[3][0]: %d\r\n", rectangle_position[3][0]);
                    break;
                case 0x11: // 第4个点Y坐标
                    rectangle_position[3][1] = (value);
                    printf("rectangle_position[3][1]: %d\r\n", rectangle_position[3][1]);
                    break;
                case 0x12: // 第1个点X坐标
                    rectangle_position[0][0] = (value);
                    printf("rectangle_position[0][0]: %d\r\n", rectangle_position[0][0]);
                    break;
                case 0x13: // 第1个点Y坐标
                    rectangle_position[0][1] = (value);
                    printf("rectangle_position[0][1]: %d\r\n", rectangle_position[0][1]);
                    break;
                case 0x14: // 第2个点X坐标
                    rectangle_position[1][0] = (value);
                    printf("rectangle_position[1][0]: %d\r\n", rectangle_position[1][0]);
                    break;
                case 0x15: // 第2个点Y坐标
                    rectangle_position[1][1] = (value);
                    printf("rectangle_position[1][1]: %d\r\n", rectangle_position[1][1]);
                    break;
                case 0x16: // 第3个点X坐标
                    rectangle_position[2][0] = (value);
                    printf("rectangle_position[2][0]: %d\r\n", rectangle_position[2][0]);
                    break;
                case 0x17: // 第3个点Y坐标
                    rectangle_position[2][1] = (value);
                    printf("rectangle_position[2][1]: %d\r\n", rectangle_position[2][1]);
                    break;
                case 0x18: // 标记矩形坐标接收完成
                    rectangle_position_receive_complete = 1;
                    break;
                default:
                    break;
                }

                frame_receiving = 0;
                frame_index = 0;
            }
        }
    }
}

/** ***************************************************************************
 * @brief 激光坐标协议解包
 * @param buff 循环缓冲区指针
 * @return uint8_t 解析结果
 */
uint8_t Lazer_Protocol_Parse(lwrb_t *buff)
{
    uint8_t data;
    uint8_t frame_index = 0;
    uint8_t frame_receiving = 0;
    while (lwrb_read(buff, &data, 1) > 0)
    {
        if (!frame_receiving)
        {
            if (data == LAZ_FRAME_HEAD1)
            {
                frame_receiving = 1;
                frame_index = 0;
                frame_buffer[frame_index] = data;
                frame_index++;
            }
        }
        else if (frame_index == 1)
        {
            if (data == LAZ_FRAME_HEAD2)
            {
                frame_buffer[frame_index] = data;
                frame_index++;
            }
            else
            {
                frame_receiving = 0;
                frame_index = 0;
            }
        }
        else
        {
            frame_buffer[frame_index] = data;
            frame_index++;
            if (frame_index > FRAME_MAX_LEN)
            {
                frame_receiving = 0;
                frame_index = 0;
            }
            else if (data == FRAME_TAIL && frame_index == FRAME_MAX_LEN)
            {
                printf("激光帧接收完成: ");
                for (int i = 0; i < FRAME_MAX_LEN; i++)
                    printf("%02X ", frame_buffer[i]);
                printf("\r\n");

                uint8_t id = frame_buffer[2];
                uint8_t h = frame_buffer[3];
                uint8_t l = frame_buffer[4];
                uint8_t z = frame_buffer[5];
                int value = h + l + z;

                printf("ID: 0x%02X, Value: %d\r\n", id, value);

                switch (id)
                {
                case 0x00: // 激光X坐标
                    lazer_position[0] = (value);
                    printf("lazer_position[0]: %d\r\n", lazer_position[0]);
                    break;
                case 0xFF: // 激光Y坐标
                    lazer_position[1] = (value);
                    printf("lazer_position[1]: %d\r\n", lazer_position[1]);
                    lazer_position_receive_complete = 1;
                    break;
                default:
                    break;
                }

                frame_receiving = 0;
                frame_index = 0;
            }
        }
    }
}

void Lazer_Reset(void)
{
    lazer_control_hal.Move_Lazer_x(servo_list[SERVO_0X].mid_pwm);
    lazer_control_hal.Move_Lazer_y(servo_list[SERVO_0Y].mid_pwm);
}

void Lazer_Set_Middle_Position(void)
{
    servo_list[SERVO_0X].mid_pwm = servo_list[SERVO_0X].pwm;
    servo_list[SERVO_0Y].mid_pwm = servo_list[SERVO_0Y].pwm;
}