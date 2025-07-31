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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

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
void stepper1_reset_wrapper(void)
{
  ZDT_Stepper_trigger_zero_return(zdt_stepper_motors[STEPPER_YAW], 0, 0);
}

// Stepper 2 回零函数包装器
void stepper2_reset_wrapper(void)
{
  ZDT_Stepper_trigger_zero_return(zdt_stepper_motors[STEPPER_PITCH], 0, 0);
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
  MX_TIM13_Init();
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
  ZDT_Stepper_Enable(zdt_stepper_motors[0], 1, 0);
  ZDT_Stepper_Enable(zdt_stepper_motors[1], 1, 0);

  /* ----------------------------------- 菜单 ----------------------------------- */
  // 初始化OLED显示
  astra_ui_driver_init();
  astra_init_core();

  astra_list_item_t *Control = astra_new_list_item("Control");
  astra_push_item_to_list(astra_get_root_list(), Control);

  astra_list_item_t *Settings = astra_new_list_item("Settings");
  astra_push_item_to_list(astra_get_root_list(), Settings);

  /* ---------------------------------- Flag ---------------------------------- */

  astra_list_item_t *Flag = astra_new_list_item("Flag");
  astra_push_item_to_list(Settings, Flag);

  astra_push_item_to_list(Flag, astra_new_switch_item("UART RX En", &uart_rx_enable_flag));
  astra_push_item_to_list(Flag, astra_new_switch_item("PID Update", &pid_update_flag));
  astra_push_item_to_list(Flag, astra_new_switch_item("Stepper Update", &stepper_update_flag));
  astra_push_item_to_list(Flag, astra_new_switch_item("PID DEBUG", &pid_debug_flag));

  uart_rx_enable_flag = 1; // 允许接收数据
  pid_debug_flag = 1;      // 允许PID调试

  /* ----------------------------------- PID ---------------------------------- */

  astra_list_item_t *PID = astra_new_list_item("PID");
  astra_push_item_to_list(Settings, PID);

  // PID参数滑块
  astra_push_item_to_list(PID, astra_new_slider_item(
                                   "Kp", &pid_kp_slider_val, 1, 0, 100));
  astra_push_item_to_list(PID, astra_new_slider_item(
                                   "Ki", &pid_ki_slider_val, 1, 0, 100));
  astra_push_item_to_list(PID, astra_new_slider_item(
                                   "Kd", &pid_kd_slider_val, 1, 0, 100));
  astra_push_item_to_list(PID, astra_new_slider_item(
                                   "tau", &pid_tau_slider_val, 1, 0, 100));
  astra_push_item_to_list(PID, astra_new_slider_item(
                                   "limMin", &pid_lim_min_slider_val, 1, -100, 0));
  astra_push_item_to_list(PID, astra_new_slider_item(
                                   "limMax", &pid_lim_max_slider_val, 1, 0, 100));
  astra_push_item_to_list(PID, astra_new_slider_item(
                                   "limMinInt", &pid_lim_min_int_slider_val, 1, -50, 0));
  astra_push_item_to_list(PID, astra_new_slider_item(
                                   "limMaxInt", &pid_lim_max_int_slider_val, 1, 0, 50));
  astra_push_item_to_list(PID, astra_new_slider_item(
                                   "SampleTime", &pid_sample_time_slider_val, 1, 1, 100));

  // 同步PID滑块到浮点数
  sync_pid_slider_to_float();
  Stepper_PID_Parameters_Init(&yaw_pid);   // 同步航向轴PID参数
  Stepper_PID_Parameters_Init(&pitch_pid); // 同步俯仰轴PID参数
  // 初始化PID控制器
  PIDController_Init(&yaw_pid);   // 初始化航向轴PID
  PIDController_Init(&pitch_pid); // 初始化俯仰轴PID

  astra_list_item_t *Stepper = astra_new_list_item("Stepper");
  astra_push_item_to_list(Settings, Stepper);

  astra_push_item_to_list(Stepper, astra_new_user_item("Stepper 1 RST", stepper1_reset_wrapper, NULL, NULL));
  astra_push_item_to_list(Stepper, astra_new_user_item("Stepper 2 RST", stepper2_reset_wrapper, NULL, NULL));

  HAL_TIM_Base_Start_IT(&htim7); // 10ms 定时器

  ebtn_APP_Key_INIT();

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

    if (in_astra == 0)
    {
      if (pid_update_flag == 1)
      {
        u8g2_DrawStr(&u8g2, 0, 10, "UART RX Done"); // 在OLED上显示接收完成信息
      }
      else
      {
        u8g2_DrawStr(&u8g2, 0, 10, "UART RX Not Done"); // 在OLED上显示接收未完成信息
      }
    }

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
