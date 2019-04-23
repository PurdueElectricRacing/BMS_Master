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
*     Name of Function: task_fan_PWM
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
void task_fan_PWM() {
  TickType_t time_init = 0;
  uint32_t duty_percentage = 50;
  int16_t temp;
  master_tim_pwm_set_duty(periph.tim, 0);
  HAL_TIM_PWM_Start(periph.tim, TIM_CHANNEL_1);
  while (1) {
    //set PWM value based on temperature value
    temp = bms.macros.temp_avg;
    if (temp < 200) {
      duty_percentage = 0;
    } else if (temp < 250) {
      duty_percentage = 30;
    } else if (temp < 450) {
      duty_percentage = 60;
    } else if (temp < 600) {
      duty_percentage = 90;
    } else {
      duty_percentage = 100;
    }
    master_tim_pwm_set_duty(periph.tim, duty_percentage);
    vTaskDelayUntil(&time_init, FAN_PWM_RATE);
  }
}

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
void master_tim_pwm_start(TIM_HandleTypeDef* htim) {
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
void master_tim_pwm_stop(TIM_HandleTypeDef* htim) {
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
void master_tim_pwm_set_duty(TIM_HandleTypeDef* htim, uint32_t duty_percent) {
  uint32_t duty = TIM_PWM_PERIOD * duty_percent / 100;
  //change duty cycle based on what the fan needs
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, duty);
}
