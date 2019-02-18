/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bms_can.h"
#include "dcan.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WDI_Pin GPIO_PIN_0
#define WDI_GPIO_Port GPIOF
#define CHARGE_ENABLE_Pin GPIO_PIN_2
#define CHARGE_ENABLE_GPIO_Port GPIOF
#define SDC_BMS_FAULT_Pin GPIO_PIN_3
#define SDC_BMS_FAULT_GPIO_Port GPIOF
#define LPM_Pin GPIO_PIN_4
#define LPM_GPIO_Port GPIOF
#define POWER_LOSS_Pin GPIO_PIN_10
#define POWER_LOSS_GPIO_Port GPIOF
#define POWER_LOSS_EXTI_IRQn EXTI15_10_IRQn
#define ISENSE_1_Pin GPIO_PIN_1
#define ISENSE_1_GPIO_Port GPIOA
#define ISENSE_2_Pin GPIO_PIN_2
#define ISENSE_2_GPIO_Port GPIOA
#define Fan_PWM_Pin GPIO_PIN_9
#define Fan_PWM_GPIO_Port GPIOE
#define BLUE_LED_Pin GPIO_PIN_10
#define BLUE_LED_GPIO_Port GPIOE
#define GREEN_LED_Pin GPIO_PIN_11
#define GREEN_LED_GPIO_Port GPIOE
#define ORANGE_LED_Pin GPIO_PIN_12
#define ORANGE_LED_GPIO_Port GPIOE
#define RED_LED_Pin GPIO_PIN_13
#define RED_LED_GPIO_Port GPIOE
#define CHARG_RX_Pin GPIO_PIN_12
#define CHARG_RX_GPIO_Port GPIOB
#define CHARG_TX_Pin GPIO_PIN_13
#define CHARG_TX_GPIO_Port GPIOB
#define BMSCAN_RX_Pin GPIO_PIN_8
#define BMSCAN_RX_GPIO_Port GPIOA
#define DCAN_RX_Pin GPIO_PIN_11
#define DCAN_RX_GPIO_Port GPIOA
#define DCAN_TX_Pin GPIO_PIN_12
#define DCAN_TX_GPIO_Port GPIOA
#define DETECT_SDIO_Pin GPIO_PIN_0
#define DETECT_SDIO_GPIO_Port GPIOD
#define BMSCAN_TX_Pin GPIO_PIN_4
#define BMSCAN_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
