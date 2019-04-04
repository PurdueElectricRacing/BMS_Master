/*
 * coulomb_counting.h
 *
 *  Created on: Apr 4, 2019
 *      Author: JoshS
 */

#ifndef COULOMB_COUNTING_H_
#define COULOMB_COUNTING_H_

#include "stm32f7xx_hal.h"
#include "bms.h"

#define TIM_PWM_PERIOD    10000

void task_coulomb_counting();

#endif /* COULOMB_COUNTING_H_ */
