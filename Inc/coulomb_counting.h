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
#include "adc_isense.h"

//Unit conversion
#define HOUR_TO_SECOND                  60 * 60
#define MILLISECOND_TO_SECOND           1000

//Temperature cutoffs
#define	TEMPERATURE_CUTOFF_1		    -10
#define	TEMPERATURE_CUTOFF_2		    0
#define	TEMPERATURE_CUTOFF_3		    25
#define	TEMPERATURE_CUTOFF_4		    45
#define	TEMPERATURE_CUTOFF_5		    60

void task_coulomb_counting();

#endif /* COULOMB_COUNTING_H_ */
