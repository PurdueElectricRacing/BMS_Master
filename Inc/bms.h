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
#include "dcan.h"

#define NUM_SLAVES		2	//how many slaves are hooked up to the system
#define NUM_VTAPS			6 //number of voltage taps per module
#define NUM_TEMP			2	//number of thermistors per module

//Delays
#define DELAY_SLAVE_CON	500 / portTICK_RATE_MS //time between checking if all slaves are connected

//RTOS Defines
#define HEARTBEAT_STACK_SIZE 128
#define HEARTBEAT_PRIORITY  1
#define BMS_MAIN_STACK_SIZE 128
#define BMS_MAIN_PRIORITY   1

#define HEARTBEAT_RATE  750 / portTICK_RATE_MS
#define BMS_MAIN_RATE		20 / portTICK_RATE_MS

//Fault Masks
//Byte 0
#define FAULT_CHARGE_EN_MASK		  0x01
#define FAULT_DISCHARGE_EN_MASK   0x02
#define FAULT_OVERVOLT_MASK			  0x04
#define FAULT_UNDERVOLT_MASK		  0x08
#define FAULT_OVERTEMP_MASK			  0x10
#define FAULT_UNDERTEMP_MASK		  0x20
#define FAULT_CHARGE_EN_SHIFT		  0
#define FAULT_DISCHARGE_EN_SHIFT  1
#define FAULT_OVERVOLT_SHIFT		  2
#define FAULT_UNDERVOLT_SHIFT		  3
#define FAULT_OVERTEMP_SHIFT		  4
#define FAULT_UNDERTEMP_SHIFT		  5

//Byte 1 - 7
#define FAULT_MODL_CON_MASK				0x01
#define FAULT_MODL_TEMP_MASK			0x02
#define FAULT_MODL_VOLT_MASK			0x04
#define FAULT_MODH_CON_MASK				0x10
#define FAULT_MODH_TEMP_MASK			0x20
#define FAULT_MODH_VOLT_MASK			0x40
#define FAULT_MODL_CON_SHIFT			0
#define FAULT_MODL_TEMP_SHIFT			1
#define FAULT_MODL_VOLT_SHIFT			2
#define FAULT_MODH_CON_SHIFT			4
#define FAULT_MODH_TEMP_SHIFT			5
#define FAULT_MODH_VOLT_SHIFT			6

//Macros
#define bitwise_or(shift, mask, logical) (((uint8_t) logical << shift) | mask)

enum bms_master_state {
  INIT        = 0,
	BMS_CONNECT = 1,
  NORMAL_OP   = 2,
  ERROR_BMS   = 3,
  SOFT_RESET  = 4,
  SHUTDOWN    = 5
};

enum power_state {
	POWER_ON = 0,
	POWER_OFF = 1
}powercmd_t;

enum flag_state {
	ASSERTED = 1,
	DEASSERTED = 0
}flag_t;

enum fault_state {
	FAULTED = 0,
	NORMAL = 1
}fault_t;

enum success_state {
	SUCCESSFUL = 0,
	FAILURE = 1
}success_t;

typedef struct {
	SemaphoreHandle_t sem;
	uint16_t data[NUM_SLAVES][NUM_VTAPS];
}vtap_t;

typedef struct {
	SemaphoreHandle_t sem;
	uint16_t data[NUM_SLAVES][NUM_VTAPS];
}temp_t;

typedef struct {
	fault_t connected; //is it communicating over can?
	fault_t temp_sens; //is slavex connected to temp sens
	fault_t volt_sens; //is slavex connected to volt sens
}slave_faults;

typedef struct {
	flag_t clear;				//clear all current faults
	fault_t charg_en;			//is charging enabled
	fault_t discharg_en;	//is discharge enabled
	fault_t overvolt;			//was there an over volt
	fault_t undervolt;		//was there an under volt
	fault_t overtemp;			//are any cells over temp?
	fault_t undertemp;
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

  vtap_t						vtaps; //2d array holding all voltage values
  temp_t						temp;	//2d array holding all temperature values

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
extern SD_HandleTypeDef  hsd1;
extern DMA_HandleTypeDef hdma_sdmmc1_tx;
extern DMA_HandleTypeDef hdma_sdmmc1_rx;
extern TIM_HandleTypeDef htim1;

void task_bms_main();
void initBMSobject();
void initRTOSObjects();
void task_heartbeat();
success_t power_cmd_slaves(powercmd_t poweron);
success_t slaves_not_connected();
success_t send_faults();


#endif /* BMS_H_ */
