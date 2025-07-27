/** ***************************************************************************
 * @File Name: ebtn_APP.c
 * @brief 基于easy_button库的事件回调按键处理应用层实现
 * 在此处修改按键触发事件
 * @note 为了实现多平台适配，本应用层的实现注入了硬件抽象相关回调函数，需要依赖硬件抽象层ebtn_Custom_HAL.c/h。同时需要依赖配置文件ebtn_Custom_Config.c/h
 * @note 由于直接使用了事件回调进行按键检测，层级间仍然存在一定耦合，按键触发事件仍需在此应用层文件实现
 * @ref ebtn_Custom_HAL.c/h
 * @credit : bowenstudy / easy_button https://github.com/bobwenstudy/easy_button
 * @Author : Sighthesia
 * @Version : 1.2.0
 * @Creat Date : 2025-03-01
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes : 添加回调函数机制，降低各层级耦合度，从而实现多平台适配
 * @Modifi Date : 2025-05-06
 */

#include "ebtn_APP.h"
#include "ebtn_Custom_HAL.h"
#include "ebtn_Custom_Config.h"

/** ***************************************************************************
 * @brief  获取按键状态回调函数
 * @param  btn: easy_button按键结构体指针
 * @return uint8_t: 按键状态，0表示未按下，1表示按下
 */
static uint8_t ebtn_APP_Get_State(struct ebtn_btn *btn)
{
    // 通过配置列表结构体查找按键硬件信息
    for (int i = 0; i < key_list_size; i++)
    {
        if (key_list[i].key_id == btn->key_id)
        {
            uint8_t pin_state = ebtn_Custom_HAL.Read_Pin(key_list[i].gpio_port, key_list[i].gpio_pin);
            // 根据有效电平转换
            if (key_list[i].active_level == pin_state)
            {
                pin_state = 1;
            }
            else
            {
                pin_state = 0;
            }
            return pin_state;
        }
    }

    /* ----------------------------- 此处可自定义按键状态获取方式 ----------------------------- */
    // 可自定义特殊按键的检测方式，如矩阵按键的检测

    return 0; // 未找到按键ID，返回0
}

/* ------------------------------- 此函数可修改按键触发事件 ------------------------------- */

/** ***************************************************************************
 * @brief  按键事件处理函数回调函数
 * @param  btn: easy_button按键结构体指针
 * @param  evt: 事件类型
 */
static void ebtn_APP_Event_Handler(struct ebtn_btn *btn, ebtn_evt_t evt)
{
    switch (btn->key_id) // 按键ID
    {
        /* ---------------------------------- EC11 ---------------------------------- */
    case EC11_KEY:
        /* ---------------------------------- 按下按键时 --------------------------------- */
        if (evt == EBTN_EVT_ONPRESS)
        {
        }
        /* ---------------------------------- 松开按键时 --------------------------------- */
        else if (evt == EBTN_EVT_ONRELEASE)
        {
        }
        /* ----------------------------- 短按按键时（可获取连击次数） ----------------------------- */
        else if (evt == EBTN_EVT_ONCLICK)
        {
            /* ----------------------------------- 单击时 ---------------------------------- */
            if (btn->click_cnt == 1)
            {
                // --------------------------------- 此处修改单击事件 --------------------------------
                astra_selector_jump_to_selected_item();
                printf("单击事件触发\n");
            }
            /* ----------------------------------- 双击时 ---------------------------------- */
            else if (btn->click_cnt == 2)
            {
                // --------------------------------- 此处修改双击事件 --------------------------------
                astra_selector_exit_current_item();
                printf("双击事件触发\n");
            }
            /* ----------------------------------- 三击时 ---------------------------------- */
            else if (btn->click_cnt == 3)
            {
            }
        }
        /* ------------------------- 长按达到最短时间（默认600ms），触发长按计数时 ------------------------ */
        else if (evt == EBTN_EVT_KEEPALIVE)
        {
            /* ------------------------------- 长按计数到达指定值时 ------------------------------- */
            if (btn->keepalive_cnt == 1)
            {
                if (in_astra == false)
                {
                    in_astra = true;
                }
                else if (in_astra == true)
                {
                    in_astra = false;
                }
                printf("长按事件触发\n");
            }
        }
        break;
    }
}

/* -------------------------------- 初始化和处理函数 -------------------------------- */

/** ***************************************************************************
 * @brief easy_button初始化
 * @note  在主函数中调用
 */
void ebtn_APP_Key_INIT(void)
{
    // 初始化easy_button库
    ebtn_init(btn_array, btn_array_size,
              btn_combo_array, btn_combo_array_size,
              ebtn_APP_Get_State, ebtn_APP_Event_Handler);

    /* ---------------------------- 此处向组合键结构体数组静态添加按键 --------------------------- */
    // 结构体数组索引位与配置中组合键结构体数组中对应的组合键ID枚举值一致
    ebtn_combo_btn_add_btn(&btn_combo_array[0], KEY_1);
}

/* --------------------------------- 自定义配置部分结束 -------------------------------- */

/** ***************************************************************************
 * @brief  处理按键事件
 * @note  定期调用，建议以20ms为周期
 * @note  Tick时基为1ms
 */
void ebtn_APP_Key_Process(void)
{
    ebtn_process(ebtn_Custom_HAL.Get_Tick()); // 获取时间处理按键事件
}
