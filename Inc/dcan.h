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
#define ID_GUI_CMD					0x620
#define ID_PARAM_SET				0x621

//Master -> GUI
#define ID_GUI_VOLT_MSG					0x605
#define ID_GUI_TEMP_MSG					0x606
#define ID_GUI_OCV_MSG					0x607
#define ID_GUI_IR_MSG						0x608
#define	ID_GUI_ERROR_MSG				0x609
#define ID_GUI_MACRO_MSG				0x60A // {SOC, Pack Volt, Pack Current, High Temp}
#define ID_GUI_SD_READ_MSG			0x60B
#define ID_GUI_SD_TOGGLE_MSG		0x60C //toggles when msg starts/ends

//rates
#define DCAN_TX_RATE 50 / portTICK_RATE_MS //send at 20Hz
#define DCAN_RX_RATE 50 / portTICK_RATE_MS //send at 20Hz

//Timeouts

//TX RTOS
#define DCAN_TX_STACK_SIZE   128
#define DCAN_TX_Q_SIZE       8
#define DCAN_TX_PRIORITY     1

//RX Process RTOS
#define DCAN_RX_STACK_SIZE   128
#define DCAN_RX_Q_SIZE       8
#define DCAN_RX_PRIORITY     1

//structures

//Functions
void dcan_filter_init();
void task_txDcan();
void task_DcanProcess();

#endif /* DCAN_H_ */
