/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "TIM.h"
#include "u8g2.h"
#include "u8g2_Custom_HW_I2C_HAL.h"
#include "astra_ui_drawer.h"
#include "ebtn_APP.h"
#include "Encoder_EXTI.h"
#include "lwrb.h"
#include "ZDT_Stepper.h"
#include "Stepper_PID.h"
#include "USART.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* --------------------------------- 用户控制标志位（菜单配置） -------------------------------- */
_Bool menu_stepper_tracking_enable = 0;  // 菜单控制：步进电机跟踪使能 (PID跟踪主开关)
_Bool menu_uart_data_receive_enable = 1; // 菜单控制：UART数据接收使能 (自动设置为开启)
_Bool menu_pid_params_update_enable = 1; // 菜单控制：PID参数实时更新使能 (自动设置为开启)
_Bool menu_pid_calculation_debug = 0;    // 菜单控制：PID计算过程调试输出 (默认关闭)
_Bool menu_stepper_command_debug = 0;    // 菜单控制：步进电机指令调试输出 (简化，不在菜单显示)
_Bool menu_pid_params_debug = 0;         // 菜单控制：PID参数更新调试输出 (简化，不在菜单显示)
_Bool menu_lazer_enable = 0;             // 菜单控制：手动激光使能

/* --------------------------------- 调试信息控制变量 -------------------------------- */
_Bool debug_coordinate_info = 0;    // 调试：坐标差值信息显示 (默认关闭)
_Bool debug_rotation_detection = 0; // 调试：旋转检测与PID切换信息显示 (默认关闭)
_Bool debug_pid_laser_info = 0;     // 调试：PID激光控制信息显示 (默认关闭)

/* --------------------------------- 旋转检测相关变量 -------------------------------- */
_Bool rotate_detect = 0;         // 旋转检测使能标志位
_Bool rotation_active = 0;       // 旋转激活状态标志位
_Bool rotation_target_found = 0; // 旋转过程中发现目标标志位

/* --------------------------------- PID激光控制相关变量 -------------------------------- */
_Bool pid_laser_enable = 1;          // PID激光控制使能标志位
float pid_laser_threshold = 5.0f;    // PID激光触发阈值（坐标偏差）
uint32_t pid_laser_duration_ms = 50; // PID激光持续时间（毫秒）
_Bool pid_laser_active = 0;          // PID激光激活状态标志位
uint32_t pid_laser_start_time = 0;   // PID激光开始时间

/* --------------------------------- 自定义部分结束 -------------------------------- */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#ifdef __GNUC__
int _write(int fd, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
#endif

// Stepper 1 回零函数包装器
void stepper1_reset(void)
{
  ZDT_Stepper_trigger_zero_return(zdt_stepper_motors[0], 0, 0);
}

// Stepper 2 回零函数包装器
void stepper2_reset(void)
{
  ZDT_Stepper_trigger_zero_return(zdt_stepper_motors[1], 0, 0);
}

void Stepper_All_Reset(void)
{
  ZDT_Stepper_trigger_zero_return(zdt_stepper_motors[0], 0, 0);
  ZDT_Stepper_trigger_zero_return(zdt_stepper_motors[1], 0, 0);
}

void Stepper_All_Disable(void)
{
  ZDT_Stepper_Enable(zdt_stepper_motors[0], ZDT_STEPPER_DISABLE, ZDT_STEPPER_SYNC_DISABLE);
  ZDT_Stepper_Enable(zdt_stepper_motors[1], ZDT_STEPPER_DISABLE, ZDT_STEPPER_SYNC_DISABLE);
}

void Stepper_All_Enable(void)
{
  ZDT_Stepper_Enable(zdt_stepper_motors[0], ZDT_STEPPER_ENABLE, ZDT_STEPPER_SYNC_DISABLE);
  ZDT_Stepper_Enable(zdt_stepper_motors[1], ZDT_STEPPER_ENABLE, ZDT_STEPPER_SYNC_DISABLE);
}

/** ***************************************************************************
 * @brief  启动旋转检测：开始360度旋转搜索目标
 */
void Start_Rotate_Detection(void)
{
  if (!rotation_active)
  {
    rotation_active = 1;
    rotation_target_found = 0;

    if (debug_rotation_detection)
    {
      printf("[Rotation] ========== 启动360°旋转检测 ==========\r\n");
      printf("[Rotation] 旋转速度: 1.0 RPM | 旋转角度: 360°\r\n");
      printf("[Rotation] PID跟踪已临时禁用，开始搜索目标...\r\n");
    }

    // 开始360度旋转
    ZDT_Stepper_Set_position(zdt_stepper_motors[0],
                             ZDT_STEPPER_CCW,
                             1.0f,                      // 1 RPM速度
                             180.0f,                    // 180度
                             ZDT_STEPPER_REL_POS_MODE,  // 相对位置模式
                             ZDT_STEPPER_SYNC_DISABLE); // 不同步
  }
}

/** ***************************************************************************
 * @brief  停止旋转检测：停止电机并切换到PID模式
 */
void Stop_Rotate_Detection(void)
{
  if (rotation_active)
  {
    rotation_active = 0;

    // 停止yaw轴电机
    ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);

    if (debug_rotation_detection)
    {
      printf("[Rotation] ========== 目标检测成功！ ==========\r\n");
      printf("[Rotation] 停止旋转电机，切换到PID跟踪模式\r\n");
      printf("[Rotation] 当前坐标: X=%d, Y=%d\r\n",
             rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
      printf("[Rotation] PID跟踪已启用，开始精确瞄准...\r\n");
    }

    // 启用PID跟踪
    menu_stepper_tracking_enable = 1;
    rotation_target_found = 1;
  }
}

/** ***************************************************************************
 * @brief  系统状态显示函数
 */
void System_Status_Display(void)
{
  static uint32_t last_status_time = 0;
  uint32_t current_time = HAL_GetTick();

  // 每5秒输出一次系统状态 (仅当有调试开关打开时)
  if ((current_time - last_status_time) >= 5000 &&
      (debug_coordinate_info || debug_rotation_detection || debug_pid_laser_info))
  {
    printf("\r\n========== 系统状态 ==========\r\n");
    printf("PID跟踪: %s | 自动旋转: %s | 自动激光: %s | 手动激光: %s\r\n",
           menu_stepper_tracking_enable ? "开启" : "关闭",
           rotate_detect ? "开启" : "关闭",
           pid_laser_enable ? "开启" : "关闭",
           menu_lazer_enable ? "开启" : "关闭");
    printf("目标坐标: X=%.1f, Y=%.1f | 当前坐标: X=%d, Y=%d\r\n",
           yaw_pid_target, pitch_pid_target,
           rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
    printf("旋转状态: %s | 激光状态: %s\r\n",
           rotation_active ? "旋转中" : "停止",
           pid_laser_active ? "发射中" : "待机");
    printf("==============================\r\n\r\n");

    last_status_time = current_time;
  }
}

/** ***************************************************************************
 * @brief  处理旋转检测逻辑
 */
void Process_Rotate_Detection(void)
{
  // 检查是否收到UART数据包（目标检测到）
  if (rotation_active && uart_packet_received)
  {
    if (debug_rotation_detection)
    {
      printf("[Rotation] 检测到UART数据包！矩形坐标: X=%d, Y=%d\r\n",
             rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
      printf("[Rotation] 准备停止旋转并切换到PID模式...\r\n");
    }
    Stop_Rotate_Detection();
  }
}

/** ***************************************************************************
 * @brief  PID激光检查与发射
 */
void PID_Laser_Check_And_Fire(void)
{
  if (!pid_laser_enable || menu_stepper_tracking_enable == 0)
  {
    return; // PID激光未启用或PID跟踪未启用
  }

  // 计算当前坐标偏差的总距离
  float yaw_error = fabsf(yaw_pid_target - (float)rectangle_coordinate_arr[0]);
  float pitch_error = fabsf(pitch_pid_target - (float)rectangle_coordinate_arr[1]);
  float total_error = sqrtf(yaw_error * yaw_error + pitch_error * pitch_error);

  // 调试信息：实时显示偏差信息
  if (debug_pid_laser_info)
  {
    static uint32_t last_debug_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 每100ms输出一次调试信息
    if ((current_time - last_debug_time) >= 100)
    {
      printf("[PID Laser] 偏差分析: Yaw=%.2f, Pitch=%.2f, 总偏差=%.2f, 阈值=%.2f\r\n",
             yaw_error, pitch_error, total_error, pid_laser_threshold);
      last_debug_time = current_time;
    }
  }

  // 如果偏差小于阈值且激光未激活，则触发激光
  if (total_error < pid_laser_threshold && !pid_laser_active)
  {
    pid_laser_active = 1;
    pid_laser_start_time = HAL_GetTick();

    if (debug_pid_laser_info)
    {
      printf("[PID Laser] ========== 激光发射触发！ ==========\r\n");
      printf("[PID Laser] 目标精度达到要求: %.2f < %.2f\r\n", total_error, pid_laser_threshold);
      printf("[PID Laser] 激光持续时间: %lums\r\n", pid_laser_duration_ms);
      printf("[PID Laser] 当前坐标: X=%d, Y=%d\r\n",
             rectangle_coordinate_arr[0], rectangle_coordinate_arr[1]);
    }
  }
}

/** ***************************************************************************
 * @brief  PID激光处理：检查激光持续时间并关闭
 */
void PID_Laser_Process(void)
{
  if (pid_laser_active)
  {
    uint32_t current_time = HAL_GetTick();
    if ((current_time - pid_laser_start_time) >= pid_laser_duration_ms)
    {
      pid_laser_active = 0;

      if (debug_pid_laser_info)
      {
        printf("[PID Laser] ========== 激光发射结束 ==========\r\n");
        printf("[PID Laser] 实际持续时间: %lums\r\n", current_time - pid_laser_start_time);
        printf("[PID Laser] 激光关闭，等待下次触发条件...\r\n");
      }
    }
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 串口接收初始化
  lwrb_init(&uart3_rx_buffer, uart3_rx_buffer_data, sizeof(uart3_rx_buffer_data));

  UART3_Start_Receive_IT();

  // 使能电机
  ZDT_Stepper_Init();
  ZDT_Stepper_Enable(zdt_stepper_motors[0], 1, ZDT_STEPPER_SYNC_DISABLE);
  ZDT_Stepper_Enable(zdt_stepper_motors[1], 1, ZDT_STEPPER_SYNC_DISABLE);

  /* ----------------------------------- 菜单结构优化 ----------------------------------- */
  // 初始化OLED显示
  astra_ui_driver_init();
  astra_init_core();

  /* ---------------------------------- 主控制菜单 ---------------------------------- */
  astra_list_item_t *Control = astra_new_list_item("Control");
  astra_push_item_to_list(astra_get_root_list(), Control);

  // 核心功能控制
  astra_push_item_to_list(Control, astra_new_switch_item("PID Tracking", &menu_stepper_tracking_enable));
  astra_push_item_to_list(Control, astra_new_switch_item("Auto Rotate", &rotate_detect));
  astra_push_item_to_list(Control, astra_new_switch_item("Auto Laser", &pid_laser_enable));
  astra_push_item_to_list(Control, astra_new_switch_item("Manual Laser", &menu_lazer_enable));

  // 目标坐标设置
  astra_push_item_to_list(Control, astra_new_slider_item("Target X", &yaw_pid_target_slider_val, 1, -100, 100));
  astra_push_item_to_list(Control, astra_new_slider_item("Target Y", &pitch_pid_target_slider_val, 1, -100, 100));

  // 激光参数
  astra_push_item_to_list(Control, astra_new_slider_item("Laser Precision", &pid_laser_threshold_slider_val, 1, 1, 100));
  astra_push_item_to_list(Control, astra_new_slider_item("Laser Duration", &pid_laser_duration_slider_val, 1, 1, 500));

  /* ---------------------------------- 参数调节菜单 ---------------------------------- */
  astra_list_item_t *Parameters = astra_new_list_item("Parameters");
  astra_push_item_to_list(astra_get_root_list(), Parameters);

  // PID参数调节
  astra_list_item_t *PID_Tuning = astra_new_list_item("PID Tuning");
  astra_push_item_to_list(Parameters, PID_Tuning);

  astra_push_item_to_list(PID_Tuning, astra_new_slider_item("Kp", &pid_kp_slider_val, 1, 0, 100));
  astra_push_item_to_list(PID_Tuning, astra_new_slider_item("Ki", &pid_ki_slider_val, 1, 0, 100));
  astra_push_item_to_list(PID_Tuning, astra_new_slider_item("Kd", &pid_kd_slider_val, 1, 0, 100));
  astra_push_item_to_list(PID_Tuning, astra_new_slider_item("Deadzone", &stepper_deadzone_slider_val, 1, 0, 50));

  // 电机控制
  astra_list_item_t *Motor_Control = astra_new_list_item("Motor Control");
  astra_push_item_to_list(Parameters, Motor_Control);

  astra_push_item_to_list(Motor_Control, astra_new_user_item("Reset All", Stepper_All_Reset, NULL, NULL));
  astra_push_item_to_list(Motor_Control, astra_new_user_item("Enable All", Stepper_All_Enable, NULL, NULL));
  astra_push_item_to_list(Motor_Control, astra_new_user_item("Disable All", Stepper_All_Disable, NULL, NULL));

  /* ---------------------------------- 调试菜单 ---------------------------------- */
  astra_list_item_t *Debug = astra_new_list_item("Debug");
  astra_push_item_to_list(astra_get_root_list(), Debug);

  // 调试开关
  astra_push_item_to_list(Debug, astra_new_switch_item("Show Coordinates", &debug_coordinate_info));
  astra_push_item_to_list(Debug, astra_new_switch_item("Show Rotation", &debug_rotation_detection));
  astra_push_item_to_list(Debug, astra_new_switch_item("Show Laser", &debug_pid_laser_info));
  astra_push_item_to_list(Debug, astra_new_switch_item("PID Debug", &menu_pid_calculation_debug));

  // 高级PID参数（调试用）
  astra_list_item_t *Advanced_PID = astra_new_list_item("Advanced PID");
  astra_push_item_to_list(Debug, Advanced_PID);

  astra_push_item_to_list(Advanced_PID, astra_new_slider_item("tau", &pid_tau_slider_val, 1, 0, 100));
  astra_push_item_to_list(Advanced_PID, astra_new_slider_item("limMin", &pid_lim_min_slider_val, 1, -100, 0));
  astra_push_item_to_list(Advanced_PID, astra_new_slider_item("limMax", &pid_lim_max_slider_val, 1, 0, 100));
  astra_push_item_to_list(Advanced_PID, astra_new_slider_item("limMinInt", &pid_lim_min_int_slider_val, 1, -50, 0));
  astra_push_item_to_list(Advanced_PID, astra_new_slider_item("limMaxInt", &pid_lim_max_int_slider_val, 1, 0, 50));
  astra_push_item_to_list(Advanced_PID, astra_new_slider_item("SampleTime", &pid_sample_time_slider_val, 1, 1, 100));

  /* ---------------------------------- 默认设置 ---------------------------------- */
  // 核心功能默认设置
  menu_uart_data_receive_enable = 1; // 默认允许接收数据
  menu_pid_params_update_enable = 1; // 默认允许PID参数更新

  // 调试信息默认关闭（减少干扰）
  debug_coordinate_info = 0;
  debug_rotation_detection = 0;
  debug_pid_laser_info = 0;
  menu_pid_calculation_debug = 0;

  // 同步PID滑块到浮点数
  sync_pid_slider_to_float();
  Stepper_PID_Parameters_Init(&yaw_pid);   // 同步航向轴PID参数
  Stepper_PID_Parameters_Init(&pitch_pid); // 同步俯仰轴PID参数
  // 初始化PID控制器
  PIDController_Init(&yaw_pid);   // 初始化航向轴PID
  PIDController_Init(&pitch_pid); // 初始化俯仰轴PID

  ebtn_APP_Key_INIT();

  HAL_TIM_Base_Start_IT(&htim7); // 10ms 定时器

  char msg[20] = {};
  sprintf(msg, "启动时间: %dms.", get_ticks());
  astra_push_pop_up(msg, 1500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    if (Encoder_Is_Increase())
    {
      if (in_astra == 1)
      {
        astra_selector_go_next_item();
      }
    }
    else if (Encoder_Is_Decrease())
    {
      if (in_astra == 1)
      {
        astra_selector_go_prev_item();
      }
    }

    if (menu_lazer_enable || pid_laser_active)
    {
      HAL_GPIO_WritePin(Lazer_GPIO_Port, Lazer_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(Lazer_GPIO_Port, Lazer_Pin, GPIO_PIN_RESET);
    }

    // 旋转检测控制逻辑
    if (rotate_detect && !rotation_active)
    {
      // 启动旋转检测时，临时禁用PID跟踪
      menu_stepper_tracking_enable = 0;

      if (debug_rotation_detection)
      {
        printf("[Rotation] 用户启动旋转检测模式\r\n");
      }

      Start_Rotate_Detection();
    }
    else if (!rotate_detect && rotation_active)
    {
      // 手动停止旋转检测
      if (debug_rotation_detection)
      {
        printf("[Rotation] 用户手动停止旋转检测\r\n");
      }

      Stop_Rotate_Detection();
    }

    // 处理旋转检测过程中的目标发现
    Process_Rotate_Detection();

    // 处理PID激光控制
    PID_Laser_Process();

    // 系统状态显示 (定期显示)
    System_Status_Display();

    oled_clear_buffer();

    astra_ui_widget_core();
    astra_ui_main_core();

    oled_send_buffer();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
