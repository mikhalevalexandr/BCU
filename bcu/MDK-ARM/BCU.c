#include "main.h"
#include "math.h"
#include "BCU.h"
void SetPoint_Setting ()
{
	SetPoint = COEF_K_BRAKE_TO_VALVE*(NeededBrakePressure)+COEF_B_BRAKE_TO_VALVE;
	SetPoint_Volts = SetPoint/MAX_BAR_VALVE_INFO*MAX_V_VALVE_INFO;
}
void SetPoint_to_ValveDutyCycle ()
{
	ValveDutyCycle[0] = -SetPoint/MAX_BAR_VALVE_INFO*MAX_DUTYCYCLE_CLOCKS_FOR_MAX_VALVE+REAL_MAX_DUTYCYCLE_CLOCKS_FROM_STM;
}
// For experiment
void SetPoint_Volts_to_ValveDutyCycle ()
{
	ValveDutyCycle[0] = -SetPoint_Volts/MAX_V_VALVE_INFO*MAX_DUTYCYCLE_CLOCKS_FOR_MAX_VALVE+REAL_MAX_DUTYCYCLE_CLOCKS_FROM_STM;
}
uint32_t MedianArray(uint32_t *arr, size_t size) {
    // Находим min и max массива,
    // чтобы получить наибольшую разность;
		uint32_t median = 0;
    uint32_t max = -1;
    uint32_t min = arr[0];
    uint32_t left_sum = 0;
    uint32_t right_sum = 0;
    for (size_t i = 1;i < size - 1;i++) {  // Минимальная разность		 
		left_sum = 0;
		right_sum = 0;
		for (size_t j = 0;j < i;j++)
		    left_sum += arr[j];
		for (size_t k = i + 1;k < size;k++)
		    right_sum += arr[k];
		if (abs(left_sum - right_sum) >= max)
		    max = abs(left_sum - right_sum);  
    } 
    for (size_t i = 1;i < size - 1;i++) {
	left_sum = 0;
	right_sum = 0;
	for (size_t j = 0;j < i;j++)
	    left_sum += arr[j];
	for (size_t k = i + 1;k < size;k++)
	    right_sum += arr[k];
	if (abs(left_sum - right_sum) <= max)
	    max = abs(left_sum - right_sum);
    }
    for (size_t i = 1;i < size - 1;i++) {
	left_sum = 0;
	right_sum = 0;
	for (size_t j = 0;j < i;j++)
	    left_sum += arr[j];
	for (size_t k = i + 1;k < size;k++)
	    right_sum += arr[k];
	if (abs(left_sum - right_sum) == max)
	    median = i;
    }
	return arr[median];
}
void SetPoint_Setting_PID ()
{
    errorCurrent_PID = NeededBrakePressure - PressureINFO_Bars;
    if ((((Ki_PID * errorIntegral_PID) <= MAX_BAR_PRESSURE_INFO) && (errorCurrent_PID >= 0)) ||
        (((Ki_PID * errorIntegral_PID) >= 0) && (errorCurrent_PID < 0)))
    {
      errorIntegral_PID += errorCurrent_PID;
    }
    errorDifferential_PID = (errorCurrent_PID - errorPrevious_PID) / (PID_PERIOD);
    SetPoint = Kp_PID * errorCurrent_PID + Ki_PID * errorIntegral_PID + Kd_PID * errorDifferential_PID;
		SetPoint_Volts = SetPoint/MAX_BAR_VALVE_INFO*MAX_V_VALVE_INFO;
		
//    if (SetPoint < 0)
//    {
//      SetPoint = 0;
//    }
//    if (SetPoint > MAX_BAR_VALVE_INFO)
//    {
//      SetPoint = MAX_BAR_VALVE_INFO;
//    }
    errorPrevious_PID = errorCurrent_PID;
}