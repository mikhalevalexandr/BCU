/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BCU.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern uint32_t ADC[MEASURING_NUMBER_ALL_CHANNELS];//buffer for reading battery voltage
extern float mcuVoltage;// STM32 power supply voltage
extern float PressureINFO_Volts; // Signal from Pressure sensor in Volts (0-5V)
extern float PressureINFO_Bars; // Pressure in braking system (Bars)
extern uint16_t ValveDutyCycle[1]; // PWM duty cycle for Valve setpoint value
extern float SetPoint; // Setpoint value for Valve in Bars (0-6 Bar)
extern float NeededBrakePressure;
extern float ActualPoint;
extern float ActualPoint_Bars;
extern float Kp_PID; 
extern float Ki_PID; 
extern float Kd_PID; 
extern float errorPrevious_PID;
extern float errorCurrent_PID;
extern float errorIntegral_PID;
extern float errorDifferential_PID;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PressureINFO_Pin GPIO_PIN_0
#define PressureINFO_GPIO_Port GPIOA
#define ValveINFO_Pin GPIO_PIN_1
#define ValveINFO_GPIO_Port GPIOA
#define ValvePWM_Pin GPIO_PIN_8
#define ValvePWM_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

//#define MINIMUM_BATTERY_VOLTAGE 			3.2 /*Voltage devider before ADC. For example battery voltage is 5V, there is 2.5V on stm ADC */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/