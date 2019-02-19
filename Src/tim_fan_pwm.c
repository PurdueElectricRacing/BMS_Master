/*
 * tim_fan_pwm.c
 *
 *  Created on: Feb 12, 2019
 *      Author: JoshS
 */

#include "tim_fan_pwm.h"

/***************************************************************************
*
*     Function Information
*
*     Name of Function: master_tim_pwm_start
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. TIM_HandleTypeDef*, htim
*
*      Global Dependents:
*       1. None
*
*     Function Description: Get ADC value when conversion finish and
*     and restart after a time delay
***************************************************************************/
void master_tim_pwm_start(TIM_HandleTypeDef* htim)
{
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: master_tim_pwm_stop
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. TIM_HandleTypeDef*, htim, timer handle
*
*      Global Dependents:
*       1. None
*
*     Function Description: Get ADC value when conversion finish and
*     and restart after a time delay
***************************************************************************/
void master_tim_pwm_stop(TIM_HandleTypeDef* htim)
{
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: master_tim_pwm_set_duty
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. TIM_HandleTypeDef*, htim, timer handle
*       2. uint32_t, duty_percent, 42% duty_percent = 42
*
*      Global Dependents:
*       1. None
*
*     Function Description: Change duty cycle of fan pwm signal for
*     controlling fan speed
***************************************************************************/
void master_tim_pwm_set_duty(TIM_HandleTypeDef* htim, uint32_t duty_percent)
{
	uint32_t duty = TIM_PWM_PERIOD * duty_percent / 100;
	//change duty cycle based on what the fan needs
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, duty);
}
