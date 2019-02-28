/*
 * sd_card.c
 *
 *  Created on: Feb 20, 2019
 *      Author: raymo
 */
#include "sd_card.h"

void task_sd_card() {
  init_sd_card();
  /*Code not needed for demo
  TickType_t time_init = 0;
  while (1) {
    time_init = xTaskGetTickCount();
    vTaskDelayUntil(&time_init, SD_CARD_RATE);  // @matt what is this supposed to be?
  }
  */
  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
  uint8_t rtext[100];                                   /* File read buffer */
  FIL MyFile;     /* File object */
  HAL_Delay(100);
  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) == FR_OK) {
		if (f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
			return;/* 'STM32.TXT' file Open for write Error */
		} else {
			/*##-5- Write data to the text file ################################*/
			res = f_write(&MyFile, wtext, sizeof(wtext), (void*)&byteswritten);
			if ((byteswritten == 0) || (res != FR_OK)) {
				return;/* 'STM32.TXT' file Write or EOF Error */
			} else {
				/*##-6- Close the open text file #################################*/
				f_close(&MyFile);
				/*##-7- Open the text file object with read access ###############*/
				if (f_open(&MyFile, "STM32.TXT", FA_READ) != FR_OK) {
					return;/* 'STM32.TXT' file Open for read Error */
				} else {
					/*##-8- Read data from the text file ###########################*/
					res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);
					if ((bytesread == 0) || (res != FR_OK)) { /* EOF or Error */
						return;/* 'STM32.TXT' file Read or EOF Error */
					} else {
						/*##-9- Close the open text file #############################*/
						f_close(&MyFile);
						/*##-10- Compare read data with the expected data ############*/
						if ((bytesread != byteswritten)) {
							return;/* Read data is different from the expected data */
						}
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
					}
				}
			}
		}
	}
  FATFS_UnLinkDriver(SDPath);
}

void init_sd_card() {
  FATFS SDFatFs;  /* File system object for SD disk logical drive */
  char SDPath[4]; /* SD disk logical drive path */
  uint8_t workBuffer[_MAX_SS];
  
  if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0) {
    if (f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK) {
      return;// error handling
    } else {
      if (f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer)) != FR_OK) {
        ;// error handling
      } else {
        return;
      }
    }
  } else {
    return;// error handling
  }
}
