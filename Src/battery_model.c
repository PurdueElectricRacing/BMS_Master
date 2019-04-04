/*
 * battery_model.c
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
  //inputs
  //Temporary variables
  int32_t d_DOD;

  while (1) {


    vTaskDelayUntil(&time_init, FAN_PWM_RATE);
  }
}
