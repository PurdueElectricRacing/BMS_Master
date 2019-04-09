/*
 * battery_model.c
 *
 *  Created on: Apr 4, 2019
 *      Author: JoshS
 */

#include "battery_model.h"
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

//Parameters definition
//VL = output voltage of battery
//IL = load current
//Vpa = voltage drop across Rpa
//Rpa = effective resistance characterizing electrochemical polarization
//Cpa = effective capacitance characterizing electrochemical polarization
//Vpc = voltage drop across Rpc
//Rpc = effective resistance characterizing concentration polarization
//Cpc = effective capacitance characterizing concentration polarization
//Voc = open circuit voltage
//Ro = ohmic resistance
//dt = time step
//tpa = time constant characterizing electrochemical polarization
//tpa = Rpa * Cpa
//tpc = time constant characterizing concentration polarization
//Tpc = Rpc * Cpc

void task_battery_model() {
  TickType_t time_init = 0;
  //CONSTANTS
  //TODO: inputs
  int IL;
  int dt;
  int Ipa;
  int Ipc;
  int tpa;
  int tpc;
  int Rpa;
  int Rpc;
  int Ro;
  int Voc;
  //Temporary variables
  int32_t d_DOD;

  while (1) {
      //Update input values
      //TODO: Rpa, Cpa, Rpc, Cpc value varies based on SOC
      //TODO: addIpa, Ipc;


    vTaskDelayUntil(&time_init, COULOMB_COUNTING_RATE);
  }
}
