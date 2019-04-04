/*
 * coulomb_counting.c
 *
 *  Created on: Apr 4, 2019
 *      Author: JoshS
 */
#include "coulomb_counting.h"

/***************************************************************************
*
*     Function Information
*
*     Name of Function: task_coulomb_counting
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. None
*
*      Global Dependents:
*       1. None
*
*     Function Description:
*
***************************************************************************/
void task_coulomb_counting() {
  TickType_t time_init = 0;
  uint32_t duty_percentage = 50;
  //CONSTANTS
  int32_t V_min = LIMIT_VOLT_LOW;
  int32_t V_max = LIMIT_VOLT_HIGH;
  //inputs
  int32_t I_instant;
  int32_t V_instant;
  int32_t T_instant;
  int32_t C_rated;
  int32_t SOC;
  int32_t SOH;
  int32_t DOD;
  int32_t dt;
  //Temporary variables
  int32_t d_DOD;
  int32_t a;

  while (1) {
	//Discharge mode
	if (I_instant > 0)
	{
        // Reset SOH
        if (V_instant < V_min)
        {
            SOH = DOD;
        }
        // Current integration
        else
        {
            d_DOD = I_instant * dt / C_rated;
            DOD = DOD + d_DOD;
            SOC = SOH - DOD;
        }
	}
	//Charge mode
	else if (I_instant < 0)
	{
        if (V_instant > V_max)
        {
            SOH = SOC;
        }
        else
        {
            d_DOD = I_instant * dt / C_rated;
            DOD = DOD - d_DOD;
            SOC = SOH - DOD;
        }
	}
	//Open circuit
	else
	{
		//TODO: self discharge consideration
	}

	//TODO: Thermal effects
	if (T_instant <= -10)
	{
        a = 0.6;
	}
    else if (T_instant <= 5)
    {
        a = 0.8;
    }
    else if (T_instant <= 25)
    {
        a = 1;
    }
    else if (T_instant <= 45)
    {
        a = 0.95;
    }
    else if (T_instant <= 60)
    {
        a = 0.9;
    }
    else
    {
        a = 0.8;
    }
    SOC = SOC * a;


    vTaskDelayUntil(&time_init, FAN_PWM_RATE);
  }
}
