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
  //TODO: outputs
  int VL;
  //TODO: inputs
  int IL;
  int Ro;
  int Voc;
  int Rpa;
  int Rpc;
  //only for Method 1
  int dt;
  int Ipa;
  int Ipc;
  int tpa;
  int tpc;
  //only for Method 2
  int Vpa;
  int Vpc;
  int Cpa;
  int Cpc;

  //Temporary variables

  while (1) {
      // Method 1: using time constant
      //Update input values
      //TODO: Rpa, tpa, Rpc, tpc value varies based on SOC
      //TODO: add Ipa, Ipc to macro for battery characterization;

      //Calculation for Method 1
      Ipa = (1 - ((1 - exp(-dt / tpa)) / (dt / tpa))) * IL
          + (((1 - exp(-dt / tpa)) / (dt / tpa)) - exp(-dt / tpa)) * IL
          + exp(-dt / tpa) * Ipa;

      Ipc = (1 - ((1 - exp(-dt / tpc)) / (dt / tpc)) ) * IL
          + (((1 - exp(-dt / tpc)) / (dt / tpc)) - exp(-dt / tpc)) * IL
          + exp(-dt / tpc) * Ipc;

      Voc = VL + Ro * IL + Rpa * Ipa + Rpc * Ipc;

      // Method 2: using estimated effective resistance and effective capacitance
      //Update input values
      //TODO: Rpa, Cpa, Rpc, Cpc value varies based on SOC
      //TODO: add Ipa, Ipc to macro for battery characterization;

      //Calculation for Method 2
//      Vpa = - (Vpa / (Rpa * Cpa)) + (IL / Cpa);
//      Vpc = - (Vpc / (Rpc * Cpc)) + (IL / Cpc);
//      Voc = VL + Vpa + Vpc + IL * Ro;

      // Method 3: using piecewise linear model with user inputed data

    vTaskDelayUntil(&time_init, COULOMB_COUNTING_RATE);
  }
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: update_dp_model_parameters
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
void update_dp_model_parameters(){

}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: LUT_SOC_OCV
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
void LUT_SOC_OCV(){

}
