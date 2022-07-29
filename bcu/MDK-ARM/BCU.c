#include "main.h"
#include "math.h"
#include "BCU.h"
/*******************************************************************************
* Function Name  : SetPoint_Setting_PID
* Description    : It sets new SetPoint(pressure on valve) without PID. 
										Don`t use this func in final project

* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void SetPoint_Setting ()
//{
//	SetPoint = COEF_K_BRAKE_TO_VALVE*(NeededBrakePressure)+COEF_B_BRAKE_TO_VALVE;
//	SetPoint_Volts = SetPoint/MAX_BAR_VALVE_INFO*MAX_V_VALVE_INFO;
//}
/*******************************************************************************
* Function Name  : SetPoint_to_ValveDutyCycle
* Description    : Converts SetPoint in Bars to appropriate Duty Cycle 

* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetPoint_to_ValveDutyCycle ()
{
	ValveDutyCycle[0] = -SetPoint/MAX_BAR_VALVE_INFO*MAX_DUTYCYCLE_CLOCKS_FOR_MAX_VALVE+REAL_MAX_DUTYCYCLE_CLOCKS_FROM_STM;
}
/*******************************************************************************
* Function Name  : SetPoint_Setting_PID
* Description    : It compares real pressure in brakes with needed pressure, calculates errors and new SetPoint(pressure on valve)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*Don`t change it*/
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
		

    errorPrevious_PID = errorCurrent_PID;
}