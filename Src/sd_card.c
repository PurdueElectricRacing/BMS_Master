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
    vTaskDelayUntil(&time_init, SD_CARD_RATE);  // @matt what is this supposed to be?
  }
}

void init_sd_card() {

}
