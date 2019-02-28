/*
 * sd_card.h
 *
 *  Created on: Feb 20, 2019
 *      Author: raymo
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

#include "bms.h"
#include "sd_diskio.h"
#include "ff_gen_drv.h"
#include "fatfs.h"

#define SD_CARD_RATE       500 / portTICK_RATE_MS // @matt

void init_sd_card();
void task_sd_card();

#endif /* SD_CARD_H_ */
