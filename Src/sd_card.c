/*
 * sd_card.c
 *
 *  Created on: Feb 20, 2019
 *      Author: raymo
 */
#include "sd_card.h"

void task_sd_card() {
  init_sd_card();
  TickType_t time_init = 0;
  while (1) {
    time_init = xTaskGetTickCount();
    vTaskDelayUntil(&time_init, SD_CARD_RATE);	// @matt what is this supposed to be?
  }
}

void init_sd_card(){
	FATFS SDFatFs;  /* File system object for SD disk logical drive */
	FIL MyFile;     /* File object */
	char SDPath[4]; /* SD disk logical drive path */
	uint8_t workBuffer[_MAX_SS];

	if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0){
	    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
	    {
	    	;// error handling
	    }
	    else{
	    	/*##-4- Create and Open a new text file object with write access #####*/
			if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
				;// error handling
			}
	    }
	}
	else{
		;// not linked
	}
}
