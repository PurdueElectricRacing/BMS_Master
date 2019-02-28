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
#include "bms.h"

#define ISENSE_CHANNEL_1			75
#define ISENSE_CHANNEL_2			500
#define ISENSE_MAX					4095
#define CURRENT_VALUE_OFFSET		10

void task_getIsense();
void master_adc_isense_start(ADC_HandleTypeDef* hadc);
void master_adc_isense_stop(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc);

#endif /* ADC_ISENSE_H_ */
