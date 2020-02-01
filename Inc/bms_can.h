/*
 * bms_can.h
 *
 *  Created on: Feb 11, 2019
 *      Author: Matt Flanagan
 */
#ifndef BMS_CAN_H_
#define BMS_CAN_H_

//Includes
#include "bms.h"
#include "FreeRTOS.h"
#include "semphr.h"
//Constants
#define NO_MESSAGES_RECV      0

//ID Master -> Slave
#define ID_BMS_WAKEUP         0x600
#define ID_MAS_CONFIG         0x601
#define ID_MAS_PASSIVE        0x603 //passive balancing Message
#define ID_WDAWG              0x604

//ID Slave -> Master
#define ID_SLAVE_ACK          0x640
#define ID_SLAVE_FAULT        0x641
#define ID_SLAVE_VOLT         0x642
#define ID_SLAVE_TEMP         0x643

//rates
#define CAN_TX_RATE 5 / portTICK_RATE_MS //send at 20Hz
#define CAN_RX_RATE 5 / portTICK_RATE_MS //send at 20Hz
#define WDAWG_RATE  1000 / portTICK_PERIOD_MS //every second check with one slave

//Timeouts
#define TIMEOUT         5 / portTICK_RATE_MS
#define WDAWG_TIMEOUT   (6000 / portTICK_RATE_MS) / NUM_SLAVES

//Length
#define WDAWG_LENGTH    1

//TX RTOS
#define BMSCAN_TX_STACK_SIZE   128
#define BMSCAN_TX_Q_SIZE       8
#define BMSCAN_TX_PRIORITY     1

//RX Process RTOS
#define BMSCAN_RX_STACK_SIZE   128
#define BMSCAN_RX_Q_SIZE       20
#define BMSCAN_RX_PRIORITY     1

//WDawg RTOS
#define WDAWG_STACK_SIZE    128
#define WDAWG_PRIORITY      1

//Macros

//structures
typedef struct {
  uint32_t StdId;    /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId;    /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];   /*!< Contains the data to be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

} CanTxMsgTypeDef;

typedef struct {
  uint32_t StdId;       /*!< Specifies the standard identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId;       /*!< Specifies the extended identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE;         /*!< Specifies the type of identifier for the message that will be received.
                             This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR;         /*!< Specifies the type of frame for the received message.
                             This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC;         /*!< Specifies the length of the frame that will be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];      /*!< Contains the data to be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FMI;         /*!< Specifies the index of the filter the message stored in the mailbox passes through.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FIFONumber;  /*!< Specifies the receive FIFO number.
                             This parameter can be CAN_FIFO0 or CAN_FIFO1 */

} CanRxMsgTypeDef;

//Global Variables

//Functions
void bms_can_filter_init();
void task_txBmsCan();
void task_Slave_WDawg();
void task_BmsCanProcess();


#endif /* BMS_CAN_H_ */
