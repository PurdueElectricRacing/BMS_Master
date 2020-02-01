/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

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
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define ISENSE_1_Pin GPIO_PIN_0
#define ISENSE_1_GPIO_Port GPIOA
#define ISENSE_2_Pin GPIO_PIN_1
#define ISENSE_2_GPIO_Port GPIOA
#define POWER_LOSS_Pin GPIO_PIN_2
#define POWER_LOSS_GPIO_Port GPIOA
#define PACK_VOLT_Pin GPIO_PIN_4
#define PACK_VOLT_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define Fan_PWM_Pin GPIO_PIN_9
#define Fan_PWM_GPIO_Port GPIOE
#define BMSCAN_TX_Pin GPIO_PIN_13
#define BMSCAN_TX_GPIO_Port GPIOB
#define DETECT_SDIO_Pin GPIO_PIN_10
#define DETECT_SDIO_GPIO_Port GPIOD
#define GREEN_LED_Pin GPIO_PIN_12
#define GREEN_LED_GPIO_Port GPIOD
#define ORANGE_LED_Pin GPIO_PIN_13
#define ORANGE_LED_GPIO_Port GPIOD
#define RED_LED_Pin GPIO_PIN_14
#define RED_LED_GPIO_Port GPIOD
#define BLUE_LED_Pin GPIO_PIN_15
#define BLUE_LED_GPIO_Port GPIOD
#define DCAN_RX_Pin GPIO_PIN_11
#define DCAN_RX_GPIO_Port GPIOA
#define DCAN_TX_Pin GPIO_PIN_12
#define DCAN_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define GPIO_WWDG_EN_Pin GPIO_PIN_5
#define GPIO_WWDG_EN_GPIO_Port GPIOD
#define WDI_Pin GPIO_PIN_6
#define WDI_GPIO_Port GPIOD
#define SDC_BMS_FAULT_Pin GPIO_PIN_7
#define SDC_BMS_FAULT_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define BMSCAN_RX_Pin GPIO_PIN_5
#define BMSCAN_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
