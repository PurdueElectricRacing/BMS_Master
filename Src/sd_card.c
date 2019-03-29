/*
 * sd_card.c
 *
 *  Created on: Feb 20, 2019
 *      Author: raymo
 */
#include "sd_card.h"

void task_sd_card() {
	uint32_t wbytes;
	uint8_t wtext[] = "text to write logical disk";
	uint8_t workBuffer[_MAX_SS];
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) == FR_OK) {
		if (f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer))
				== FR_OK) {
			if (f_open(&SDFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
				if (f_write(&SDFile, wtext, sizeof(wtext), (void *) &wbytes) == FR_OK) {
					f_close(&SDFile);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				}
			}
		}
	}
	FATFS_UnLinkDriver(SDPath);

	int a = 0;
  TickType_t time_init = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  while (1) {
    time_init = xTaskGetTickCount();
    if (a == 0){
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    	a = 1;
    }
    else{
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    	a = 0;
    }
    vTaskDelayUntil(&time_init, SD_CARD_RATE);
  }
	/*
	uint32_t wbytes;
	uint8_t wtext[] = "text to write logical disk";
	uint8_t workBuffer[_MAX_SS];
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) == FR_OK) {
		if (f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer))
				== FR_OK) {
			if (f_open(&SDFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
				if (f_write(&SDFile, wtext, sizeof(wtext), (void *) &wbytes) == FR_OK) {
					f_close(&SDFile);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				}
			}
		}
	}
	FATFS_UnLinkDriver(SDPath);
	*/
}
