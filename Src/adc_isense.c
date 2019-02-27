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
*     Name of Function: heartbeat
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. None
*
*      Global Dependents:
*       1. None
*
*     Function Description: Toggles a gpio at HEARTBEAT_RATE to satisfy the
*     watchdog timer
*
***************************************************************************/
void task_getIsense() {
  TickType_t time_init = 0;
  uint32_t adc_val;
  int32_t current_val;
  uint16_t toggle = 0;
  while (1) {
    time_init = xTaskGetTickCount();
    //check if the ADC queue is empty
    if (xQueuePeek(bms.q_adc_isense, &adc_val, TIMEOUT) == pdTRUE) {
      xQueueReceive(bms.q_adc_isense, &adc_val, TIMEOUT);  //actually take item out of queue
      //check if data is from channel 1 or 2
      if (toggle == 0)
      {
    	//data from channel 1
    	toggle = 1;
    	//convert current value
    	current_val = (adc_val * 2 * ISENSE_CHANNEL_1 / ISENSE_MAX) - ISENSE_CHANNEL_1;
    	//check discharging or charging
    	if(current_val > 0)
    	{
    	  //discharging
		  //check current limit
          if(adc_val > LIMIT_DISCHARG)
          {
        	//set error flag
          }
    	}
    	else
    	{
      	  //charging
          //check current limit
          if(adc_val > LIMIT_CHARG)
          {
          	//set error flag
          }
    	}
      }
      else
      {
    	//data from channel 2
    	toggle = 0;
    	//convert current value
    	current_val = (adc_val * 2 * ISENSE_CHANNEL_2 / ISENSE_MAX) - ISENSE_CHANNEL_2;
      }
      //TODO: add current value to battery model queue
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

	// need to get data from ADC_CDR register if operation
	// mode is dual ADC mode, instead of from ADC_DR register
	//this is kinda sketch
	//	ADC_Common_TypeDef * common_hadc = ADC123_COMMON;
	//	uint32_t adc_value = common_hadc->CDR;

	//TODO: send data somewhere for processing
	//		send data using queues through freeRTOS
	xQueueGenericSendFromISR(QueueHandle_t xQUeue, const void * const pvItemToQueue
			, BaseType_t * const pxHigherPriorityTaskWoekn, const BaseType_t xCopyPosition);

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
