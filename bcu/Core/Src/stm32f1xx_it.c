/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BCU.h"
#include "pressures.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_tim1_ch1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern CAN_TxHeaderTypeDef pTxHeaderBreakPressure;
extern CAN_RxHeaderTypeDef pRxHeaderpTxHeaderBreakPressure;
extern uint32_t TxMailbox;
extern uint8_t i;
extern uint8_t xBreakPressureTX_data[2];
extern uint8_t Counter_for_PID;
extern uint16_t u16PressureINFO_BarsBuf;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	/* In this handler mcuVoltage, Actual value of pressure from Valve, Pressure in brakes are measured */
	uint16_t ADC_counter = 0;
	uint16_t arrays_counter = 0;
	uint32_t mcuVoltage_ADC_codes_arr[MEASURING_NUMBER_EACH_CHANNEL] = {0,};// MCU supply Voltage
	uint32_t PressureINFO_ADC_codes_arr[MEASURING_NUMBER_EACH_CHANNEL] = {0,};// Pressure info in brake 
	uint32_t ValveINFO_ADC_codes_arr[MEASURING_NUMBER_EACH_CHANNEL] = {0,};// Pressure info in valve 
	size_t mcuVoltage_ADC_codes_arr_size = sizeof(mcuVoltage_ADC_codes_arr)/sizeof(uint32_t);
	size_t PressureINFO_ADC_codes_arr_size = sizeof(PressureINFO_ADC_codes_arr)/sizeof(uint32_t);
	size_t ValveINFO_ADC_codes_arr_size = sizeof(ValveINFO_ADC_codes_arr)/sizeof(uint32_t);
	uint32_t mcuVoltage_ADC_code_median = 0;
	uint32_t PressureINFO_ADC_code_median = 0;
	uint32_t mcuVoltage_ADC_code_average = 0;
	uint32_t PressureINFO_ADC_code_average = 0;	
	uint32_t ValveINFO_ADC_code_average = 0;	
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
	/*McuVoltage (STM power supply Voltage) can change, but ADC_MAX(corresponding to McuVoltage) is always 0xFFF,
	therefore ADC codes( in this case corresponding to ADC_REFERENCE_VOLTAGE and batteryVoltage) can change
	But ADC_REFERENCE_VOLTAGE is stable and around 1.2 Volts,  so we can find out mcuVoltage thanks to it. 
	And then with knowledge of real McuVoltage we can find out real batteryVoltage
	ADC[0] is ADC code corresponding to ADC_REFERENCE_VOLTAGE
	ADC[1] is ADC code corresponding to PressureINFO*/
//	if (ADC[0] == 0)
//		return;
	for (ADC_counter = 0; ADC_counter<MEASURING_NUMBER_ALL_CHANNELS; ADC_counter++)
	{
		mcuVoltage_ADC_codes_arr[arrays_counter] = ADC[ADC_counter];
		ADC_counter++;
		PressureINFO_ADC_codes_arr[arrays_counter] = ADC[ADC_counter];
		ADC_counter++;		
		ValveINFO_ADC_codes_arr[arrays_counter] = ADC[ADC_counter];
		arrays_counter++;
	}	
//	mcuVoltage_ADC_code_median = MedianArray( mcuVoltage_ADC_codes_arr, mcuVoltage_ADC_codes_arr_size);
//	PressureINFO_ADC_code_median = MedianArray( PressureINFO_ADC_codes_arr, PressureINFO_ADC_codes_arr_size);
//	mcuVoltage = ADC_MAX * ADC_REFERENCE_VOLTAGE / mcuVoltage_ADC_code_median;
//	PressureINFO = (VOLT_DEV_PRESSURE * PressureINFO_ADC_code_median * mcuVoltage / ADC_MAX);
	for (arrays_counter = 0; arrays_counter<MEASURING_NUMBER_EACH_CHANNEL; arrays_counter++)
	{
		mcuVoltage_ADC_code_average+=mcuVoltage_ADC_codes_arr[arrays_counter];
		PressureINFO_ADC_code_average+=PressureINFO_ADC_codes_arr[arrays_counter];
		ValveINFO_ADC_code_average+=ValveINFO_ADC_codes_arr[arrays_counter];

	}
	mcuVoltage_ADC_code_average/=MEASURING_NUMBER_EACH_CHANNEL;
	PressureINFO_ADC_code_average/=MEASURING_NUMBER_EACH_CHANNEL;
	ValveINFO_ADC_code_average/=MEASURING_NUMBER_EACH_CHANNEL;	
//	mcuVoltage = ADC_MAX * ADC_REFERENCE_VOLTAGE / mcuVoltage_ADC_code_average;
	PressureINFO_Volts = COEF_K_PRESSURE_INFO_ADC * VOLT_DEV_PRESSURE * PressureINFO_ADC_code_average * mcuVoltage / ADC_MAX;
	ActualPoint = COEF_K_VALVE_INFO_ADC * VOLT_DEV_VALVE * ValveINFO_ADC_code_average * mcuVoltage / ADC_MAX;

	PressureINFO_Bars = MAX_BAR_PRESSURE_INFO * PressureINFO_Volts / MAX_V_PRESSURE_INFO;
	ActualPoint_Bars = MAX_BAR_VALVE_INFO * ActualPoint / MAX_V_VALVE_INFO;
	
//	for (ADC_counter = 0; ADC_counter<100; ADC_counter++)
//	{
//		mcuVoltage_arr[arrays_counter] = ADC_MAX * ADC_REFERENCE_VOLTAGE / ADC[ADC_counter];
//		PressureINFO_arr[arrays_counter] = VOLT_DEV_PRESSURE * ADC[ADC_counter+1] * mcuVoltage_arr[arrays_counter] / ADC_MAX;
//		arrays_counter++;
//		ADC_counter++;
//	}
//	MedianArray(uint32_t *arr, size_t size)
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
	/*Here we change PWM duty cycle for Valve*/
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_ch1);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
//	 SetPoint_Setting ();
//	SetPoint_Setting_PID ();
	 SetPoint_to_ValveDutyCycle ();
  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupt.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
/*This timer is used for setting SetPoint every 0,1 second*/
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
	Counter_for_PID++;
	/*When Counter_for_PID = MAX_COUNTER_PID_CLOCKS (72), it means that 0,1 second has passed*/
	if (Counter_for_PID == MAX_COUNTER_PID_CLOCKS)
	{
		Counter_for_PID = 0;
		SetPoint_Setting_PID ();
		/*In fact, It isn`t used. I don`t remember, why I wrote this next code until the end of this function*/
		/*I think, it is how i selected Kp_PID, Ki_PID, Kd_PID*/
		PressureINFO_Bars_PID_calib.arr[PressureINFO_Bars_PID_calib.counter] = PressureINFO_Bars;
		PressureINFO_Bars_PID_calib.counter++;
	}
	if (PressureINFO_Bars_PID_calib.counter == 99)
	{
		for(uint16_t k = 0; k<100; k++)
		{
			PressureINFO_Bars_PID_calib.arr[k] = 0;
		}
		PressureINFO_Bars_PID_calib.counter = 0;
	}
	/***********************************************/
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
/* This timer is made for Amesim tests. Pressures is taken from array in file pressures.c
	
	
	Don`t use whole this func in final project
	
*/	
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	/*This code used for test with preloaded values of pressure, frequency of changing is 10Hz.
		But now we use this IRQ for sending messages by CAN. Start*/
//	if (Pressurescounter<1203)
//	{
//		SetPoint_Volts = Pressures_effort_Endurance[Pressurescounter];
//		SetPoint_Volts = Voltage_effort_Step[Voltage_Testing.counter];
//		SetPoint_Volts = Voltage_effort_Proportional[Voltage_Testing.counter];
//			SetPoint_Volts = Voltage_effort_Sinus[Voltage_Testing.counter];		
//		Pressurescounter++;}
	
//	else 
//	{SetPoint_Volts = 0;
//		Pressurescounter = 0;
//		Circle++;}
	/*End*/

	/*Now this IRQ occurs with frequency 50Hz. We use it to send pressure value by CAN*/
	u16PressureINFO_BarsBuf = (uint16_t) (PressureINFO_Bars * floatBREAK_PRESSURE_DIVIDER);
	xBreakPressureTX_data[0] = (uint8_t) (u16PressureINFO_BarsBuf >> 8);
	xBreakPressureTX_data[1] = (uint8_t) (u16PressureINFO_BarsBuf & 0x00FF);
	HAL_CAN_AddTxMessage(&hcan2, &pTxHeaderBreakPressure, xBreakPressureTX_data, &TxMailbox);

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles CAN2 TX interrupt.
  */
void CAN2_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_TX_IRQn 0 */

  /* USER CODE END CAN2_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_TX_IRQn 1 */

  /* USER CODE END CAN2_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupt.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
