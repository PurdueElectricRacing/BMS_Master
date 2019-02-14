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
#include "bms_can.h"

#define NUM_SLAVES		2	//how many slaves are hooked up to the system

//RTOS Defines
#define HEARTBEAT_STACK_SIZE 128
#define HEARTBEAT_PRIORITY  1
#define BMS_MAIN_STACK_SIZE 128
#define BMS_MAIN_PRIORITY   1

#define HEARTBEAT_RATE  750 / portTICK_RATE_MS
#define BMS_MAIN_RATE		20 / portTICK_RATE_MS

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
  QueueHandle_t     q_rx_bmscan;
  QueueHandle_t     q_tx_bmscan;
  QueueHandle_t     q_rx_dcan;
  QueueHandle_t     q_tx_dcan;
  QueueHandle_t     q_rx_chargcan;
  QueueHandle_t     q_tx_chargcan;

  faults_t 					fault;

  SemaphoreHandle_t state_sem;
  enum bms_master_state state;
  //might not need q to receive since polling TBD
} bms_t;

typedef struct {
	ADC_HandleTypeDef* i_adc;
	CAN_HandleTypeDef* dcan;
	CAN_HandleTypeDef* chargcan;
	CAN_HandleTypeDef* bmscan;
	SD_HandleTypeDef*  hsd1;
	DMA_HandleTypeDef* hdma_sdmmc1_tx;
	DMA_HandleTypeDef* hdma_sdmmc1_rx;
	TIM_HandleTypeDef* tim;
}periph_t;

volatile bms_t bms;
periph_t periph;

extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;
extern SD_HandleTypeDef hsd1;
extern DMA_HandleTypeDef hdma_sdmmc1_tx;
extern DMA_HandleTypeDef hdma_sdmmc1_rx;
extern TIM_HandleTypeDef htim1;

#endif /* BMS_H_ */
