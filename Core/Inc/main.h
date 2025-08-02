/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */

  /* USER CODE END ET */

  /* Exported constants --------------------------------------------------------*/
  /* USER CODE BEGIN EC */

  /* USER CODE END EC */

  /* Exported macro ------------------------------------------------------------*/
  /* USER CODE BEGIN EM */

  /* USER CODE END EM */

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

  /* USER CODE BEGIN EFP */
  // 开环扫描模式函数声明
  void Open_Loop_Scan_Start(void);
  void Open_Loop_Scan_Stop(void);
  void Open_Loop_Scan_Process(void);
  void Laser_Fire_Trigger(void);
  void Laser_Fire_Process(void);
  _Bool Target_Center_Detected(void);
  int16_t Calculate_Target_Distance_From_Center(void);
  _Bool Detect_Target_Moving_Away(void);

  // 步进电机控制函数声明
  void Stepper_All_Reset(void);
  void Stepper_All_Disable(void);
  void Stepper_All_Enable(void);

  // 旋转检测函数声明
  void Start_Rotate_Detection(void);
  void Stop_Rotate_Detection(void);
  void Process_Rotate_Detection(void);

  // 开环扫描模式相关变量声明
  extern _Bool menu_open_loop_scan_enable;
  extern _Bool open_loop_scan_active;
  extern float scan_speed;
  extern float scan_range;
  extern uint32_t laser_fire_duration_ms;
  extern int16_t target_detect_threshold;

  // 差值检测相关变量声明
  extern int16_t last_target_distance;
  extern int16_t current_target_distance;
  extern _Bool target_moving_away;
  extern uint32_t distance_check_interval;
  extern _Bool scan_position_reached;

  // 扫描参数滑块变量声明
  extern int16_t scan_speed_slider_val;
  extern int16_t scan_range_slider_val;
  extern int16_t laser_duration_slider_val;
  extern int16_t target_threshold_slider_val;
  extern int16_t distance_check_interval_slider;

  // PID激光控制滑块变量声明
  extern int16_t pid_laser_threshold_slider_val;
  extern int16_t pid_laser_duration_slider_val;

  // 旋转检测相关变量声明
  extern _Bool rotate_detect;
  extern _Bool rotation_active;
  extern _Bool rotation_target_found;

  // PID激光控制相关变量声明
  extern _Bool pid_laser_enable;
  extern float pid_laser_threshold;
  extern uint32_t pid_laser_duration_ms;
  extern _Bool pid_laser_active;
  extern uint32_t pid_laser_start_time;

  // PID激光控制函数声明
  void PID_Laser_Check_And_Fire(void);
  void PID_Laser_Process(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Lazer_Pin GPIO_PIN_5
#define Lazer_GPIO_Port GPIOC

  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
