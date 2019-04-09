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
  //CONSTANTS
  int16_t V_min = bms.params.volt_low_lim;
  int16_t V_max = bms.params.volt_high_lim;
  //inputs
  int32_t I_instant = bms.macros.pack_i.ch1_low_current;
  uint16_t V_instant = bms.macros.pack_volt;
  int16_t T_instant = bms.macros.high_temp.val;
  int32_t C_rated;  //TODO capacity of the battery pack
  uint8_t SOC = bms.macros.soc;
  uint8_t SOH = bms.macros.soh;
  float DOD = bms.macros.dod;
  float dt = COULOMB_COUNTING_RATE * portTICK_RATE_MS / 1000;
  //Temporary variables
  float d_DOD;
  int32_t a;

  while (1) {
    //Update input values
    SOC = bms.macros.soc;
    SOH = bms.macros.soh;
    DOD = bms.macros.dod;
    I_instant = bms.macros.pack_i.ch1_low_current;
    V_instant = bms.macros.pack_volt;
    T_instant = bms.macros.high_temp.val;


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
            d_DOD = dt * I_instant / C_rated;
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

    //Thermal effects
    //TODO: currently T_instant is taken from highest temperature cell in the module
    //		should probably take the average temperature of the whole module
    //TODO: change temperature value to
    if (T_instant <= TEMPERATURE_CUTOFF_1)
    {
        a = 0.6;
    }
    else if (T_instant <= TEMPERATURE_CUTOFF_2)
    {
        a = 0.8;
    }
    else if (T_instant <= TEMPERATURE_CUTOFF_3)
    {
        a = 1;
    }
    else if (T_instant <= TEMPERATURE_CUTOFF_4)
    {
        a = 0.95;
    }
    else if (T_instant <= TEMPERATURE_CUTOFF_5)
    {
        a = 0.9;
    }
    else
    {
        a = 0.8;
    }
    SOC = SOC * a;


    vTaskDelayUntil(&time_init, COULOMB_COUNTING_RATE);
  }
}
