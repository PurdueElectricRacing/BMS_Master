/*
 * adc_isense.c
 *
 *  Created on: Feb 12, 2019
 *      Author: JoshS
 */

#include "adc_isense.h"

/***************************************************************************
*
*     Function Information
*
*     Name of Function: master_adc_isense
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
*     Function Description: Monitors can traffic to ensure Master is still in
*     comms, if it doesn't receive a message from Master within x time then
*     enter sleep mode
***************************************************************************/
void master_adc_isense_start(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Start_IT(hadc);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: HAL_ADC_ConvCpltCallback
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. ADC_HandleTypeDef*, hadc
*
*      Global Dependents:
*       1. None
*
*     Function Description: Get ADC value when conversion finish and
*     and restart after a time delay
***************************************************************************/
void master_adc_isense_stop(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Stop_IT(hadc);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: HAL_ADC_ConvCpltCallback
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. ADC_HandleTypeDef*, hadc
*
*      Global Dependents:
*       1. None
*
*     Function Description: Get ADC value when conversion finish and
*     and restart after a time delay
***************************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//stop conversion
	HAL_ADC_Stop_IT(hadc);

	//get isense ch1 and ch2 values from ADC_DR register
	uint32_t adc_value = HAL_ADC_GetValue(hadc);
	//TODO: might need to get data from ADC_CDR register
	//		instead of ADC_DR, since we in dual ADC mode
	//this is kinda sketch
	ADC_Common_TypeDef * common_hadc = ADC123_COMMON;
	uint32_t adc_value = common_hadc->CDR;
	uint16_t adc1_val = adc_value;  //lower 16bit is adc1, truncate upper 16bit
	uint16_t adc2_val = adc_value >> 16;

	//TODO: send data somewhere for processing

	//TODO: delay if needed

	//restart adc conversion
	HAL_ADC_Start_IT(hadc);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: HAL_ADC_ErrorCallback
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. ADC_HandleTypeDef*, hadc
*
*      Global Dependents:
*       1. None
*
*     Function Description: Handle any error happened with ADC
***************************************************************************/
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
	//TODO: handle error if needed
}
