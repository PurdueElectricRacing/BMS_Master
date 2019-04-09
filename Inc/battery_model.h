/*
 * battery_model.h
 *
 *  Created on: Apr 8, 2019
 *      Author: JoshS
 */

#ifndef BATTERY_MODEL_H_
#define BATTERY_MODEL_H_

#include "stm32f7xx_hal.h"
#include "bms.h"

#define TIM_PWM_PERIOD    10000

void task_battery_model();

#endif /* BATTERY_MODEL_H_ */
