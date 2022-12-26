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
struct PID_calib
{
	float arr [100];
	uint16_t counter;
} ;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern uint32_t ADC[MEASURING_NUMBER_ALL_CHANNELS];//buffer for reading battery voltage
extern float mcuVoltage;// STM32 power supply voltage
extern float PressureINFO_Volts; // Signal from Pressure sensor in Volts (0-5V)
extern float PressureINFO_Bars; // Pressure in braking system (Bars)
extern uint16_t ValveDutyCycle[1]; // PWM duty cycle for Valve setpoint value
extern float SetPoint; // Setpoint value for Valve in Bars (0-6 Bar)
extern float SetPoint_Volts; 
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
extern struct PID_calib PressureINFO_Bars_PID_calib;


extern uint16_t Pressurescounter;
extern uint8_t Circle;
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
#define BA_CAN_RX_Pin GPIO_PIN_12
#define BA_CAN_RX_GPIO_Port GPIOB
#define BA_CAN_TX_Pin GPIO_PIN_13
#define BA_CAN_TX_GPIO_Port GPIOB
#define ValvePWM_Pin GPIO_PIN_8
#define ValvePWM_GPIO_Port GPIOA
#define MAINCAN_RX_Pin GPIO_PIN_8
#define MAINCAN_RX_GPIO_Port GPIOB
#define MAINCAN_TX_Pin GPIO_PIN_9
#define MAINCAN_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

//#define MINIMUM_BATTERY_VOLTAGE 			3.2 		/*Voltage devider before ADC. For example battery voltage is 5V, there is 2.5V on stm ADC */
#define floatBREAK_PRESSURE_DIVIDER 			100.0 /*BCU receives needed pressure from CAN in value multiplicated by this */
#define floatVALVE_PRESSURE_DISCREPANCY		3.0 	/*If dicrepancy between SetPoint and ActualPoint pressure is more then 
																								  this value for u32MAX_TICKS_ERROR BCU sends error by CAN*/
#define floatBREAK_PRESSURE_DISCREPANCY		10.0 	/*If dicrepancy between needed and real break pressure is more then 
																								  this value for u32MAX_TICKS_ERROR BCU sends error by CAN*/
#define u32MAX_TICKS_ERROR								1000	/*If dicrepancy between needed and real break pressure is more then 
																								  floatVALVE_PRESSURE_DISCREPANCY BCU for u32MAX_TICKS_ERROR sends error by CAN*/																						 

#define	VALVE_ERROR                  			0x01 			/* Большая разница между устанавливаемым давлением в пневмоцилиндре и реальным*/
#define BREAK_ERROR                       0x02 		  /* Большая разница между устанавливаемым давлением в тормозной системе и реальным*/
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
