/*
 * sd_card.c
 *
 *  Created on: Feb 20, 2019
 *      Author: raymo
 */
#include "sd_card.h"

void task_sd_card() {

	uint8_t text0[] = "\n0x000, 0x0000000000000000";

	//uint8_t workBuffer[_MAX_SS];
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) == FR_OK) {
		//if (f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer))
		//		== FR_OK) {
		if (f_open(&SDFileData, "DATA.TXT", FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
			f_lseek(&SDFileData, f_size(&SDFileData));
			if (f_open(&SDFileSetting, "SETTING.TXT", FA_CREATE_ALWAYS | FA_WRITE)
					== FR_OK) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				task_sd_card_process();
			}
		}
		/*
		 if (f_open(&SDFileData, "DATA.TXT", FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
		 f_lseek(&SDFileData, f_size(&SDFileData));
		 if (f_write(&SDFileData, text0, sizeof(text0), (void *) &wbytes) == FR_OK) {
		 f_lseek(&SDFileData, f_size(&SDFileData));
		 if (f_write(&SDFileData, text1, sizeof(text1), (void *) &wbytes) == FR_OK) {
		 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		 f_close(&SDFileData);
		 }
		 }
		 if (f_open(&SDFileSetting, "SETTING.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
		 if (f_write(&SDFileSetting, sett0, sizeof(sett0), (void *) &wbytes) == FR_OK) {
		 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		 f_close(&SDFileSetting);
		 }
		 }
		 //f_unlink("STM32.TXT");// delete
		 }
		 */
		//}
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	FATFS_UnLinkDriver(SDPath);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

void task_sd_card_process() {
	uint32_t wbytes;
	char text1[] = "\n0x111, 0x1111111111111111";
	char sett0[] = "00, 0x0000\n00, 0x0000\n00, 0x0000\n00, 0x0000\n"
			"00, 0x0000\n00, 0x0000\n00, 0x0000\n00, 0x0000\n"
			"00, 0x0000\n00, 0x0000\n00, 0x0000\n00, 0x0000\n"
			"00, 0x0000\n00, 0x0000\n00, 0x0000\n00, 0x0000";
	char sett1[] = "01, 0x0001\n00, 0x0000\n00, 0x0000\n00, 0x0000\n"
			"00, 0x0000\n00, 0x0000\n00, 0x0000\n00, 0x0000\n"
			"00, 0x0000\n00, 0x0000\n00, 0x0000\n00, 0x0000\n"
			"00, 0x0000\n00, 0x0000\n00, 0x0000\n00, 0x0000";
	sdcard_t msg;
	strncpy(msg.id, "000", 3);
	strncpy(msg.data, "0000000000000000", 16);
	msg.file = 0;
	TickType_t time_init = 0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	int a = 0;
	int b = 0;
	while (1) {
		time_init = xTaskGetTickCount();
		if (xQueuePeek(bms.q_sd_card, &msg, TIMEOUT) == pdTRUE) {
			xQueueReceive(bms.q_sd_card, &msg, TIMEOUT);
			switch (msg.file) {
			case 0:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
				f_close(&SDFileData);
				f_write(&SDFileSetting, sett1, sizeof(sett1), (void *) &wbytes);
				f_close(&SDFileSetting);
				return;
				break;
			case 1:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
				if (b < 3) {
					f_lseek(&SDFileData, f_size(&SDFileData));
					f_write(&SDFileData, text1, sizeof(text1), (void *) &wbytes);
					b++;
				}
				break;
			default:
				break;
			}
		}
		if (b == 3) {
			msg.file = 0;
			xQueueSendToBack(bms.q_sd_card, &msg, TIMEOUT);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			b = 0;
		}
		if (a == 3) {
			strncpy(msg.id, "001", 3);
			strncpy(msg.data, "0123456700001111", 16);
			msg.file = 1;
			xQueueSendToBack(bms.q_sd_card, &msg, TIMEOUT);
			a = 0;
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
			a++;
		}
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		vTaskDelayUntil(&time_init, SD_CARD_RATE);
	}
}
