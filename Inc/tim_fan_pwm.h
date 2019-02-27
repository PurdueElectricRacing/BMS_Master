/*
 * tim_fan_pwm.h
 *
 *  Created on: Feb 12, 2019
 *      Author: JoshS
 */

#ifndef TIM_FAN_PWM_H_
#define TIM_FAN_PWM_H_

#include "stm32f7xx_hal.h"

#define TIM_PWM_PERIOD		10000

void master_tim_pwm_start(TIM_HandleTypeDef* htim);
void master_tim_pwm_stop(TIM_HandleTypeDef* htim);
void master_tim_pwm_set_duty(TIM_HandleTypeDef* htim, uint32_t duty_percent);

#endif /* TIM_FAN_PWM_H_ */
