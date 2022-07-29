#ifndef __BCU_H
#define __BCU_H
#include "main.h"
#include <string.h>
#define MEASURING_NUMBER_EACH_CHANNEL 100
#define MEASURING_NUMBER_ALL_CHANNELS 300
/* Private defines -----------------------------------------------------------*/
#define ADC_REFERENCE_VOLTAGE 1.20164/* Voltage of Vrefint - STM32 internal voltage reference 
																		(look stm datasheet) is around 1.2 Volts*/
#define ADC_MAX               0xFFF /* It is maximum value in 12 bit adc. It corresponds to STM power supply voltage 
																			 (VDDA or Vref+. It depends on microcontroller)*/
#define VREFINT_CAL 					1638.00/* Raw data (ADC code for 1.2 Volts)acquired at a temperature of 30 °C (± 5 °C),
																				VDDA = VREF+ = 3.0 V (± 10 mV)*/
#define VOLT_DEV_PRESSURE 			1.6045662 /*Rbottom=3.285k, Rtop=1,986k. VOLT_DEV_PRESSURE =(Rbottom+Rtop)/Rbottom=1.588235294
																					Voltage devider before ADC. For example, Pressure info is 5V,
																					there is 3.(148)V on stm ADC (PressureINFO3.3 on sheet),
																					because of resistors voltage devider on the circuit board*/
#define COEF_K_PRESSURE_INFO_ADC 							  0.989					//Coefficient k (y=kx+b) for ADC
#define COEF_K_VALVE_INFO_ADC 0.9904356												//Coefficient k (y=kx+b) for ADC
#define MAX_V_PRESSURE_INFO  	5             // 5 volts -  maximum voltage of pressure sensor`s output signal
#define MAX_V_VALVE_INFO  	10             // 10 volts -  maximum voltage of valve sensor`s output signal

#define MAX_BAR_PRESSURE_INFO 100         // 100 bar - It`s maximum pressure, wich Pressor sensor is possible to measure 
#define MAX_BAR_VALVE_INFO  	6             // 6 bar -  maximum voltage of pressure sensor`s output signal
#define MAX_DUTYCYCLE_CLOCKS_FOR_MAX_VALVE 1000         // 1000 - it is maximum number of ticks, when  duty cycle is 100%

#define REAL_MAX_DUTYCYCLE_CLOCKS_FROM_STM 1000         /* In fact, when duty cycle is 0, output voltage must be 12V, but Setpoint Voltage on 
																													Valve is 0-10V, REAL_MAX_DUTYCYCLE_CLOCKS_FROM_STM must fix it, but and with it, BCU
																														did well
																													*/
#define COEF_K_BRAKE_TO_VALVE 				0.2482075					/*Coefficient k (y=kx+b) for conversion needed pressure in brake system (NeededBrakePressure) to 
																									to pressure in valve for Pneumatic actuator in Pedal*/
#define COEF_B_BRAKE_TO_VALVE 				0					/*Coefficient b (y=kx+b) for conversion needed pressure in brake system (NeededBrakePressure) to 
																									to pressure in valve for Pneumatic actuator in Pedal*/
#define VOLT_DEV_VALVE 			3.1276596 /*Rbottom=3.285k, Rtop=1,986k. VOLT_DEV_PRESSURE =(Rbottom+Rtop)/Rbottom=1.588235294
																					Voltage devider before ADC. For example, Pressure info is 5V,
																					there is 3.(148)V on stm ADC (PressureINFO3.3 on sheet),
																					because of resistors voltage devider on the circuit board 3.1276596*/
#define PID_PERIOD	0,1								/*PID_PERIOD = MAX_COUNTER_PID_CLOCKS*(1/(APB2 timer Clocks/Prescaler/Counter Period)). It is for 
																			direvative error (I named it as errorDifferential_PID)*/	
#define MAX_COUNTER_PID_CLOCKS 72     
uint32_t MedianArray(uint32_t *arr, size_t size);
//void SetPoint_Setting ();
void SetPoint_to_ValveDutyCycle ();
void SetPoint_Setting_PID ();
#endif /* __BCU_H */