/*
 * sd_card.c
 *
 *  Created on: Feb 20, 2019
 *      Author: raymo
 */
#include "sd_card.h"

void task_sd_card() {
	//uint8_t workBuffer[_MAX_SS];
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(100);
	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) == FR_OK) {
		//if (f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer))
		//		== FR_OK) {
		HAL_Delay(100);
		if (f_open(&SDFileData, "DATA.TXT", FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
			f_lseek(&SDFileData, f_size(&SDFileData));
			HAL_Delay(100);
			if (f_open(&SDFileSetting, "SETTING.TXT", FA_CREATE_ALWAYS | FA_WRITE)
					== FR_OK) {
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				task_sd_card_process();
			}
		}
		//}
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_Delay(500);
	FATFS_UnLinkDriver(SDPath);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

void task_sd_card_process() {
	uint32_t wbytes;
	char t0[] = "\n0x111, 0x1111111111111111";
	char t1[] = "\n0x111, 0x1111111111111111";
	char setting[] = "00, 0x0001\n01, 0x0000\n02, 0x0000\n03, 0x0000\n"
			"04, 0x00000000\n05, 0x00000000\n06, 0x0000\n07, 0x0000\n"
			"08, 0x0000\n09, 0x0000\n10, 0x0000\n16, 0x0000";
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
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			xQueueReceive(bms.q_sd_card, &msg, TIMEOUT);
			switch (msg.file) {
			case 0:
				f_close(&SDFileData);
				f_write(&SDFileSetting, setting, sizeof(setting), (void *) &wbytes);
				f_close(&SDFileSetting);
				return;
				break;
			case 1:
				sprintf(t0, "\n0x%s", "000");
				sprintf(t1, ", 0x%s", "0123456701234567");
				strcpy(t0, strcat(t0, t1));
				if (b < 3) {
					f_lseek(&SDFileData, f_size(&SDFileData));
					f_write(&SDFileData, t0, sizeof(t0), (void *) &wbytes);
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
		vTaskDelayUntil(&time_init, 1000);
	}
}

void task_sd_card_get_setting(char m[], int c) {
	char f[] = "0000";
	char ff[] = "00000000";
	int x = 0;
	int d = 6 + 11 * c;
	d += 4 * (c >= 4) + 4 * (c >= 5);
	switch (c) {
	case (0):
		snprintf(f, sizeof(f) + 1, "%04x", bms.params.temp_high_lim);
		break;
	case (1):
		snprintf(f, sizeof(f) + 1, "%04x", bms.params.temp_low_lim);
		break;
	case (2):
		snprintf(f, sizeof(f) + 1, "%04x", bms.params.volt_high_lim);
		break;
	case (3):
		snprintf(f, sizeof(f) + 1, "%04x", bms.params.volt_low_lim);
		break;
	case (4):
		snprintf(ff, sizeof(ff) + 1, "%08lx", bms.params.discharg_lim);
		break;
	case (5):
		snprintf(ff, sizeof(ff) + 1, "%08lx", bms.params.charg_lim);
		break;
	case (6):
		snprintf(f, sizeof(f) + 1, "%04x", bms.params.volt_msg_rate);
		break;
	case (7):
		snprintf(f, sizeof(f) + 1, "%04x", bms.params.temp_msg_rate);
		break;
	case (8):
		snprintf(f, sizeof(f) + 1, "%04x", bms.params.ocv_msg_rate);
		break;
	case (9):
		snprintf(f, sizeof(f) + 1, "%04x", bms.params.ir_msg_rate);
		break;
	case (10):
		snprintf(f, sizeof(f) + 1, "%04x", bms.params.macro_msg_rate);
		break;
	case (11):
		x += (bms.params.volt_msg_en == ASSERTED);
		x += (bms.params.temp_msg_en == ASSERTED) << 1;
		x += (bms.params.ocv_msg_en == ASSERTED) << 2;
		x += (bms.params.ir_msg_en == ASSERTED) << 3;
		x += (bms.params.macro_msg_en == ASSERTED) << 4;
		snprintf(f, sizeof(f) + 1, "%04x", x);
		break;
	default:
		break;
	}
	if ((c == 4) || (c == 5)) {
		for (int j = 0; j < 8; j++) {
			m[d + j] = ff[j];
		}
	} else {
		for (int j = 0; j < 4; j++) {
			m[d + j] = f[j];
		}
	}
}
