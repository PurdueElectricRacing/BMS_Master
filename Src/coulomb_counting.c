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
  int32_t I_instant = bms.macros.pack_i.ch1_low_current;            //unit 0.1 A
  uint16_t V_instant = bms.macros.pack_volt;                        //unit 0.1 mV
  int16_t T_instant = bms.macros.high_temp.val;                     //unit 0.1 C
  uint8_t N_parallel = bms.cell_config.N_parallel;                  //# parallel cell
  uint8_t capacity = bms.cell_config.rated_capacity;                //unit 0.1 Ah
  uint16_t C_rated = N_parallel * capacity;                         //unit 0.1 Ah
  C_rated = C_rated * HOUR_TO_SECOND;                               //unit 0.1 As
  double SOC = bms.macros.soc / 2;                                  //unit %
  double SOH = bms.macros.soh / 2;                                  //unit %
  double DOD = bms.macros.dod / 2;                                  //unit %
  uint8_t dt = COULOMB_COUNTING_RATE * portTICK_RATE_MS;            //unit ms

  //Temporary variables
  double d_DOD;                                                     //unit %
  float a;                                                          //temperature constant

  while (1)
  {
    //Update input values
    I_instant = bms.macros.pack_i.ch1_low_current;
    if(I_instant / CURRENT_VALUE_OFFSET >= ISENSE_CHANNEL_1_MAX - ISENSE_HYSTERESIS)
    {
        I_instant = bms.macros.pack_i.ch2_high_current;
    }
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
            d_DOD = (double) dt / MILLISECOND_TO_SECOND * I_instant / C_rated / CURRENT_VALUE_OFFSET;
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
            d_DOD = (double) dt / MILLISECOND_TO_SECOND * I_instant / C_rated / CURRENT_VALUE_OFFSET;
            DOD = DOD - d_DOD;
            SOC = SOH - DOD;
        }
	}
    //Open circuit
    else
    {
        //TODO: self discharge consideration
        //Li-ion cell: 5% in 24h, then 1–2% per month (plus 3% for safety circuit)
    }

    //Thermal effects
    //TODO: currently T_instant is taken from highest temperature cell in the module
    //		should probably take the average temperature of the whole module
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
        a = 0.9;
    }
    SOC = SOC * a;

    //Update value to bms macros
    //TODO: does not take into account of the battery model SOC estimation
    bms.macros.soc = (int) SOC * 2;
    bms.macros.soh = (int) SOH * 2;
    bms.macros.dod = (int) DOD * 2;

    vTaskDelayUntil(&time_init, COULOMB_COUNTING_RATE);
  }
}
