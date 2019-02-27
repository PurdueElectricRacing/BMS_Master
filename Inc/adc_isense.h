/*
 * adc_isense.h
 *
 *  Created on: Feb 12, 2019
 *      Author: JoshS
 */

#ifndef ADC_ISENSE_H_
#define ADC_ISENSE_H_

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_adc.h"

void master_adc_isense_start(ADC_HandleTypeDef* hadc);
void master_adc_isense_stop(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc);

extern uint16_t adc1_val;
extern uint16_t adc2_val;

#endif /* ADC_ISENSE_H_ */
