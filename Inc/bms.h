/*
 * bms.h
 *
 *  Created on: Feb 11, 2019
 *      Author: Matt Flanagan
 */

#ifndef BMS_H_
#define BMS_H_

#include "main.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_can.h"
#include "FreeRTOS.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define NUM_SLAVES		2	//how many slaves are hooked up to the system

enum bms_master_state {
  INIT        = 0,
	BMS_CONNECT = 1,
  NORMAL_OP   = 2,
  ERROR_BMS   = 3,
  SOFT_RESET  = 4,
  SHUTDOWN    = 5
};

typedef struct {
	uint8_t connected; //is it communicating over can?
	uint8_t temp_sens; //is slavex connected to temp sens
	uint8_t volt_sens; //is slavex connected to volt sens
}slave_faults;

typedef struct {
	uint8_t charg_en;			//is charging enabled
	uint8_t discharg_en;	//is discharge enabled
	uint8_t overvolt;			//was there an over volt
	uint8_t undervolt;		//was there an under volt
	uint8_t overtemp;			//are any cells over temp?
	slave_faults slave[NUM_SLAVES];
	SemaphoreHandle_t error_sem;
}faults_t;

//Main BMS structure that holds can handles and all of the queues
typedef struct {
  CAN_HandleTypeDef* bms_can;
  CAN_HandleTypeDef* charg_can;
  CAN_HandleTypeDef* dcan;
  QueueHandle_t     q_rx_bmscan;
  QueueHandle_t     q_tx_bmscan;
  QueueHandle_t     q_rx_dcan;
  QueueHandle_t     q_tx_dcan;
  QueueHandle_t     q_rx_chargcan;
  QueueHandle_t     q_tx_chargcan;

  uint16_t          connected; //used to determine if connected to master

  faults_t 					fault;

  SemaphoreHandle_t state_sem;
  enum bms_master_state state;
  //might not need q to receive since polling TBD
} bms_t;


#endif /* BMS_H_ */
