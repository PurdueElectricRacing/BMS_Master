/*
 * charging.h
 *
 *  Created on: Apr 15, 2019
 *      Author: matth
 */

#ifndef CHARGING_H_
#define CHARGING_H_

#include "bms.h"

//RTOS Defines
#define CHARG_STACK_SIZE 128
#define CHARG_PRIORITY   1
#define CHARG_RATE       1000 / portTICK_RATE_MS // 1 Hz rate

//Charging Specific Defines
#define DELTA_VOLT       100 //100 uV

//Function Prototypes
void task_charging();

#endif /* CHARGING_H_ */
