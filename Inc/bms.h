/*
 * bms.h
 *
 *  Created on: Feb 11, 2019
 *      Author: Matt Flanagan
 */

#ifndef BMS_H_
#define BMS_H_

#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_can.h"
#include "FreeRTOS.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"
#include "bms_can.h"
#include "dcan.h"
#include "adc_isense.h"
#include "tim_fan_pwm.h"

#define NUM_SLAVES        1 //how many slaves are hooked up to the system
#define NUM_VTAPS         6 //number of voltage taps per module
#define NUM_TEMP          2 //number of thermistors per module
#define NUM_NORMAL_TASKS  2 //number of tasks that aren't normal

//Default Limits (Eventually should move this to the SD card)
#define LIMIT_TEMP_HIGH   600       //60 degrees C
#define LIMIT_TEMP_LOW    -200      //-20 degrees C
#define LIMIT_VOLT_HIGH   42500     //4.25 volts
#define LIMIT_VOLT_LOW    25000     //2.5 volts
#define LIMIT_DISCHARG    3000      //300 amps
#define LIMIT_CHARG       200       //20 amps

//Delays
#define DELAY_SLAVE_CON 500 / portTICK_RATE_MS //time between checking if all slaves are connected
#define DELAY_RESET     500 / portTICK_RATE_MS
#define SEND_ERROR_DELAY  1000 / portTICK_RATE_MS
#define DELAY_BMS_CONNECT 50 / portTICK_RATE_MS

//RTOS Defines
#define HEARTBEAT_STACK_SIZE        128
#define HEARTBEAT_PRIORITY          2
#define BMS_MAIN_STACK_SIZE         128
#define BMS_MAIN_PRIORITY           3
#define ERROR_CHECK_STACK_SIZE      128
#define ERROR_CHECK_RATE_PRIORITY   1

#define HEARTBEAT_RATE      750 / portTICK_RATE_MS
#define BMS_MAIN_RATE       20 / portTICK_RATE_MS
#define ERROR_CHECK_RATE    500 / portTICK_RATE_MS
#define ADC_ISENSE_RATE    500 / portTICK_RATE_MS

#define POWER_LOSS_PIN      10

//Fault Masks
//Byte 0
#define FAULT_CHARGE_EN_MASK      0x01
#define FAULT_DISCHARGE_EN_MASK   0x02
#define FAULT_OVERVOLT_MASK       0x04
#define FAULT_UNDERVOLT_MASK      0x08
#define FAULT_OVERTEMP_MASK       0x10
#define FAULT_UNDERTEMP_MASK      0x20
#define FAULT_CHARGE_EN_SHIFT     0
#define FAULT_DISCHARGE_EN_SHIFT  1
#define FAULT_OVERVOLT_SHIFT      2
#define FAULT_UNDERVOLT_SHIFT     3
#define FAULT_OVERTEMP_SHIFT      4
#define FAULT_UNDERTEMP_SHIFT     5

//Byte 1 - 7
#define FAULT_MODL_CON_MASK       0x01
#define FAULT_MODL_TEMP_MASK      0x02
#define FAULT_MODL_VOLT_MASK      0x04
#define FAULT_MODH_CON_MASK       0x10
#define FAULT_MODH_TEMP_MASK      0x20
#define FAULT_MODH_VOLT_MASK      0x40
#define FAULT_MODL_CON_SHIFT      0
#define FAULT_MODL_TEMP_SHIFT     1
#define FAULT_MODL_VOLT_SHIFT     2
#define FAULT_MODH_CON_SHIFT      4
#define FAULT_MODH_TEMP_SHIFT     5
#define FAULT_MODH_VOLT_SHIFT     6

//Macros
#define bitwise_or(shift, mask, logical) (((uint8_t) logical << shift) & mask)
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
//used to reduce a byte to a logical value based off a specified location request
#define bit_extract(mask, shift, byte) (byte & mask) >> shift
#define byte_combine(msb, lsb) ((msb << 8) | lsb)
//if it is time for the said msg to send
#define execute_broadcast(msg_rate, i) (i % (msg_rate / BROADCAST_MS) == 0)

enum bms_master_state {
  INIT        = 0,
  BMS_CONNECT = 1,
  NORMAL_OP   = 2,
  ERROR_BMS   = 3,
  SHUTDOWN    = 4,
};

typedef enum power_state {
  POWER_ON = 0,
  POWER_OFF = 1,
} powercmd_t;

typedef enum flag_state {
  ASSERTED = 1,
  DEASSERTED = 0,
} flag_t;

typedef enum fault_state {
  FAULTED = 1,
  NORMAL = 0,
} fault_t;

typedef enum {
  SUCCESSFUL = 0,
  FAILURE = 1,
} Success_t;

typedef struct {
  SemaphoreHandle_t sem;
  uint16_t data[NUM_SLAVES][NUM_VTAPS];
  uint16_t ocv[NUM_SLAVES][NUM_VTAPS];
  uint16_t ir[NUM_SLAVES][NUM_VTAPS];
} vtap_t;

typedef struct {
  SemaphoreHandle_t sem;
  int16_t data[NUM_SLAVES][NUM_TEMP];
} temp_t;

typedef struct {
  fault_t connected; //is it communicating over can?
  fault_t temp_sens; //is slavex connected to temp sens
  fault_t volt_sens; //is slavex connected to volt sens
} slave_faults;

typedef struct {
  flag_t clear;       //clear all current faults
  fault_t overall;    //is there a fault anywhere on the BMS?
  fault_t charg_en;     //is charging enabled
  fault_t discharg_en;  //is discharge enabled
  fault_t overvolt;     //was there an over volt
  fault_t undervolt;    //was there an under volt
  fault_t overtemp;     //are any cells over temp?
  fault_t undertemp;
  fault_t DOC;			//discharge over current
  fault_t COC;			//charge over current
  slave_faults slave[NUM_SLAVES];
  SemaphoreHandle_t sem;
} bmsfaults_t;

typedef struct {
  int16_t temp_high_lim; //temp
  int16_t temp_low_lim; //temp
  uint16_t volt_high_lim; //volt
  uint16_t volt_low_lim; //volt
  int16_t discharg_lim; //current
  int16_t charg_lim;  //current
  
  //broadcasts
  flag_t volt_msg_en;
  flag_t temp_msg_en;
  flag_t ocv_msg_en;  //open circuit volt
  flag_t ir_msg_en; //internal resistance
  flag_t macro_msg_en;
  
  //rates can be max BROADCAST_RATE hz and can be any integer multiple of that
  uint16_t volt_msg_rate;
  uint16_t temp_msg_rate;
  uint16_t ocv_msg_rate;  //open circuit volt
  uint16_t ir_msg_rate; //internal resistance
  uint16_t macro_msg_rate;
  
  SemaphoreHandle_t sem;
} params_t;

typedef struct {
  //todo: add a semaphore
  uint8_t soc;        //percent
  uint8_t soh;        //percent
  uint16_t pack_volt; //voltage
  int16_t pack_i;     //current
  int16_t high_temp;  //temperature
  int16_t low_temp;
  uint16_t high_volt;
  uint16_t low_volt;
} macros_t;

//Main BMS structure that holds can handles and all of the queues
typedef struct {
  QueueHandle_t     q_rx_bmscan;
  QueueHandle_t     q_tx_bmscan;
  QueueHandle_t     q_rx_dcan;
  QueueHandle_t     q_tx_dcan;
  QueueHandle_t     q_rx_chargcan;
  QueueHandle_t     q_tx_chargcan;
  
  params_t          params;
  bmsfaults_t       fault;
  
  vtap_t            vtaps; //2d array holding all voltage values
  temp_t            temp; //2d array holding all temperature values
  macros_t          macros;
  
  TaskHandle_t* normal_op_tasks[NUM_NORMAL_TASKS];
  
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
} periph_t;

typedef struct {
  TickType_t last_msg;
  SemaphoreHandle_t sem;
} WatchDawg_t;

volatile bms_t bms;
volatile WatchDawg_t wdawg[NUM_SLAVES];

periph_t periph;

extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;
extern SD_HandleTypeDef  hsd1;
extern DMA_HandleTypeDef hdma_sdmmc1_tx;
extern DMA_HandleTypeDef hdma_sdmmc1_rx;
extern TIM_HandleTypeDef htim1;

void initBMSobject(flag_t mode);
void initRTOSObjects();
void task_heartbeat();
void task_error_check();
void task_bms_main();
Success_t power_cmd_slaves(powercmd_t poweron);
Success_t slaves_connected();
Success_t send_faults();
void initRTOSNormal();
Success_t clear_faults();
void debug_lights(flag_t orange, flag_t red, flag_t green, flag_t blue);


#endif /* BMS_H_ */
