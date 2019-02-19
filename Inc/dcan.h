/*
 * dcan.h
 *
 *  Created on: Feb 14, 2019
 *      Author: Matt Flanagan
 */

#ifndef DCAN_H_
#define DCAN_H_

//Includes
#include "bms.h"

//Constants
//IDs
//GUI -> Master
#define ID_GUI_CMD          0x620
#define ID_GUI_PARAM_SET    0x621
#define ID_GUI_BMS_RESET    0x622
#define ID_GUI_PARAM_REQ    0x623

//Master -> GUI
#define ID_MASTER_VOLT_MSG      0x605
#define ID_MASTER_TEMP_MSG      0x606
#define ID_MASTER_OCV_MSG       0x607
#define ID_MASTER_IR_MSG        0x608
#define ID_MASTER_ERROR_MSG     0x609
#define ID_MASTER_MACRO_MSG     0x60A // {SOC, Pack Volt, Pack Current, High Temp}
#define ID_MASTER_SD_READ_MSG   0x60B
#define ID_MASTER_SD_TOGGLE_MS  0x60C //toggles when msg starts/ends
#define ID_MASTER_PARAM_RES     0x60D

//Masks
#define CONFIG_VOLT_MSG_MASK      0x01
#define CONFIG_TEMP_MSG_MASK      0x02
#define CONFIG_OCV_MSG_MASK       0x04
#define CONFIG_IR_MSG_MASK        0x08
#define CONFIG_MACRO_MSG_MASK     0x10
#define CONFIG_VOLT_MSG_SHIFT     0
#define CONFIG_TEMP_MSG_SHIFT     1
#define CONFIG_OCV_MSG_SHIFT      2
#define CONFIG_IR_MSG_SHIFT       3
#define CONFIG_MACRO_MSG_SHIFT    4

//can msg
#define MACRO_MSG_LENGTH          7
#define GENERIC_MSG_LENGTH        8
#define VALUES_PER_MSG            3
#define PARAM_RES_MSG_LENGTH      3

//rates
#define DCAN_TX_RATE    50 / portTICK_RATE_MS //send at 20Hz
#define DCAN_RX_RATE    50 / portTICK_RATE_MS //send at 20Hz
#define BROADCAST_MS    50
#define BROADCAST_RATE  BROADCAST_MS / portTICK_RATE_MS //fastest broadcast is 20hz
//used to keep canrxq from overflowing
#define BROADCAST_DELAY (BROADCAST_MS / 10) / portTICK_RATE_MS

//Timeouts

//TX RTOS
#define DCAN_TX_STACK_SIZE   128
#define DCAN_TX_Q_SIZE       40
#define DCAN_TX_PRIORITY     1

//RX Process RTOS
#define DCAN_RX_STACK_SIZE   128
#define DCAN_RX_Q_SIZE       8
#define DCAN_RX_PRIORITY     1

//Broadcast RTOS
#define BROADCAST_STACK_SIZE   128
#define BROADCAST_PRIORITY     1

//Macros
//if it is time for the said msg to send
#define execute_broadcast(msg_rate, i) ((msg_rate / BROADCAST_MS) % i == 0)

//expects a uint16_t type
#define extract_LSB(value) (value & 0x00FF)
#define extract_MSB(value) ((value >> 8) & 0x00FF)

//structures

//enums
typedef enum gui_cmd {
  LOG_DATA = 0,
  DELETE = 1,
  CONFIGURE = 2,
  NO_OPERATION = 3
} gui_cmd_t;

typedef enum param_cmd {
  TEMP_HIGH_LIMIT = 0,
  TEMP_LOW_LIMIT =  1,
  VOLT_HIGH_LIMIT = 2,
  VOLT_LOW_LIMIT  = 3,
  DISCHARGE_LIMIT = 4,
  CHARGE_LIMIT    = 5,
  VOLT_MSG_RATE   = 6,
  TEMP_MSG_RATE   = 7,
  OCV_MSG_RATE    = 8,
  IR_MSG_RATE     = 9,
  MACRO_MSG_RATE  = 10,
  SOC_VALUE       = 11,
  SOH_VALUE       = 12,
  PACK_VOLTAGE    = 13,
  PACK_CURRENT    = 14,
  HIGH_TEMP       = 15
} param_cmd_t;

typedef enum dcan_broadcast {
  VOLT_MSG = 0,
  TEMP_MSG = 1,
  OCV_MSG = 2,
  IR_MSG = 3,
} dcan_broadcast_t;

//Functions
void dcan_filter_init();
void task_txDcan();
void task_broadcast();
void task_DcanProcess();


#endif /* DCAN_H_ */
