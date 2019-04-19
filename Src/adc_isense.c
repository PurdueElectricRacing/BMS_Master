/*
 * adc_isense.c
 *
 *  Created on: Feb 12, 2019
 *      Author: JoshS
 */

#include "adc_isense.h"

//global
flag_t adc_toggle;

/***************************************************************************
*
*     Function Information
*
*     Name of Function: task_getIsense
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
void task_getIsense() {
  TickType_t time_init = 0;
  uint32_t adc_value;
  uint32_t adc_value2;
  int32_t current_value;
//  adc_toggle = ASSERTED;
  while (1)
  {
    time_init = xTaskGetTickCount();
    
    //poll for channel 1 adc value
    HAL_ADC_Start(periph.i_adc);
    HAL_ADC_PollForConversion(periph.i_adc, TIMEOUT);
    adc_value = HAL_ADC_GetValue(periph.i_adc);
    HAL_ADC_Stop(periph.i_adc);

    //poll for channel 2 adc value
    HAL_ADC_Start(periph.i_adc);
    HAL_ADC_PollForConversion(periph.i_adc, TIMEOUT);
    adc_value2 = HAL_ADC_GetValue(periph.i_adc);
    //stop ADC
    HAL_ADC_Stop(periph.i_adc);

    //process ADC value
    current_value = adc_value * 2 * ISENSE_CHANNEL_1_MAX * CURRENT_VALUE_OFFSET / ISENSE_MAX - ISENSE_CHANNEL_1_MAX * CURRENT_VALUE_OFFSET;
    //update measured value
    bms.macros.pack_i.ch1_low_current = current_value;
    //process ADC value
    current_value = adc_value2 * 2 * ISENSE_CHANNEL_2_MAX * CURRENT_VALUE_OFFSET / ISENSE_MAX - ISENSE_CHANNEL_2_MAX * CURRENT_VALUE_OFFSET;
    //update measured value
    bms.macros.pack_i.ch2_high_current = current_value;
    

    //determine current direction
    if (current_value < 0)
    {
        //discharging
        if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS)
        {
            if (-current_value > bms.params.discharg_lim)
            {
                //set error flag
                bms.fault.DOC = FAULTED;
            }
            else
            {
                bms.fault.DOC = NORMAL;
            }
            xSemaphoreGive(bms.fault.sem);
        }
    }
    else
    {
        //charging
        if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS)
        {
            if (current_value > bms.params.charg_lim)
            {
                //set error flag
                bms.fault.COC = FAULTED;
            }
            else
            {
                bms.fault.COC = NORMAL;
            }
            xSemaphoreGive(bms.fault.sem);
        }
    }
    
    vTaskDelayUntil(&time_init, ADC_ISENSE_RATE);
  }
}

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
void master_adc_isense_start(ADC_HandleTypeDef* hadc) {
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
void master_adc_isense_stop(ADC_HandleTypeDef* hadc) {
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  //stop conversion
  HAL_ADC_Stop_IT(hadc);
  
  //get isense ch1 and ch2 values from ADC_DR register
  uint32_t adc_value = HAL_ADC_GetValue(hadc);
  int32_t current_value;
  
  //ADC channel 1
  if (adc_toggle == ASSERTED) {
    adc_toggle = DEASSERTED;
    //process adc value
    current_value = adc_value * 2 * ISENSE_CHANNEL_1_MAX / ISENSE_MAX - ISENSE_CHANNEL_1_MAX;
    //update measured value
    bms.macros.pack_i.ch1_low_current = current_value * CURRENT_VALUE_OFFSET;
  }
  //ADC channel 2
  else {
    adc_toggle = ASSERTED;
    //process adc value
    current_value = adc_value * 2 * ISENSE_CHANNEL_2_MAX / ISENSE_MAX - ISENSE_CHANNEL_2_MAX;
    bms.macros.pack_i.ch2_high_current = current_value * CURRENT_VALUE_OFFSET;
  }
  
  // restart adc conversion
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
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {
  //TODO: handle error if needed
}
