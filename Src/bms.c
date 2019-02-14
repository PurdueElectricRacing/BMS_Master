/*
 * bms.c
 *
 *  Created on: Feb 11, 2019
 *      Author: Matt Flanagan
 */
#include "bms.h"

/***************************************************************************
*
*     Function Information
*
*     Name of Function: heartbeat
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. None
*
*      Global Dependents:
*       1. None
*
*     Function Description: Toggles a gpio at HEARTBEAT_RATE to satisfy the
*     watchdog timer
*
***************************************************************************/
void task_heartbeat() {
  TickType_t time_init = 0;
  while (1) {
    time_init = xTaskGetTickCount();
    HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
    vTaskDelayUntil(&time_init, HEARTBEAT_RATE);
  }
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: initRTOSObjects
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. None
*
*      Global Dependents:
*       1. bms
*
*     Function Description: Initializes required q's and creates the tasks
*     that freeRTOS will run
*
***************************************************************************/
void initRTOSObjects() {
  //define q's
  bms.q_rx_bmscan = xQueueCreate(BMSCAN_TX_Q_SIZE, sizeof(CanTxMsgTypeDef));
  bms.q_tx_bmscan = xQueueCreate(BMSCAN_RX_Q_SIZE, sizeof(CanRxMsgTypeDef));

  //start tasks
  xTaskCreate(task_txBmsCan, "Transmit Can", BMSCAN_TX_STACK_SIZE, NULL, BMSCAN_TX_PRIORITY, NULL);
  xTaskCreate(task_bms_main, "Main Task", BMS_MAIN_STACK_SIZE, NULL, BMS_MAIN_PRIORITY, NULL);
  xTaskCreate(task_heartbeat, "Heartbeat", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_PRIORITY, NULL);
  xTaskCreate(task_Master_WDawg, "Master WDawg", WDAWG_STACK_SIZE, NULL, WDAWG_PRIORITY, NULL);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: initBMSobject
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*       1. bms
*
*     Function Description: Initialize the BMS structure
***************************************************************************/
void initBMSobject() {
	uint8_t i = 0;
  bms.state_sem = xSemaphoreCreateBinary();
  bms.fault.charg_en = 0;
  bms.fault.discharg_en = 0;
  bms.fault.error_sem = xSemaphoreCreateBinary();
  bms.fault.overtemp = 0;
  bms.fault.overvolt = 0;
  bms.fault.undervolt = 0;

  for (i = 0; i < NUM_SLAVES; i++) {
    bms.fault.slave[i].connected = 0;
    bms.fault.slave[i].temp_sens = 0;
    bms.fault.slave[i].volt_sens = 0;
  }

  bms.state = INIT;

  xSemaphoreGive(bms.state_sem);
  xSemaphoreGive(bms.fault.error_sem);

  periph.bmscan						= &hcan3;
	periph.chargcan					= &hcan2;
	periph.dcan							= &hcan1;
	periph.hdma_sdmmc1_rx	  = &hdma_sdmmc1_rx;
	periph.hdma_sdmmc1_tx	  = &hdma_sdmmc1_tx;
	periph.hsd1						  = &hsd1;
	periph.i_adc						= &hadc1;
	periph.tim							= &htim1;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: task_bms_main
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*       1. bms
*
*     Function Description: Main execution loop of the program. Will collect all
*     processed information and send it out via can to main_bms. Will also have the
*     ability to update paramaters on the fly
***************************************************************************/
void task_bms_main() {
	initBMSobject();
	TickType_t time_init = 0;
	while (1) {
		time_init = xTaskGetTickCount();

		switch (bms.state) {
		      case INIT:

						if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
							bms.state = NORMAL_OP;
							xSemaphoreGive(bms.state_sem); //release sem
						}
		        break;
		      case BMS_CONNECT:
		      	//establish connections with all slaves
		      	break;
		      case NORMAL_OP:
		        //TODO: read from all of the sensors
		        //TODO: send data to master
		        //TODO: manage passive balancing if necessary
		        break;
		      case ERROR_BMS:
		        //TODO: handle error just send error code to the Master (let that hoe deal with it)
		        //todo: send_error();
		        if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
		          bms.state = SHUTDOWN;
		          xSemaphoreGive(bms.state_sem); //release sem
		        }
		        break;
		      case SOFT_RESET:
		      	//clear errors and move on
		      	break;
		      case SHUTDOWN:
		        //tell the WDAWG to disable
		        HAL_GPIO_WritePin(LPM_GPIO_Port, LPM_Pin, GPIO_PIN_RESET); //active low
		        //todo: disable the SPI/I2C periphs so only wakeup on can
		        //enter sleep mode and wait for interrupt to wake back up
		        if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
		          bms.state = LOW_POWER;
		          xSemaphoreGive(bms.state_sem); //release sem
		        }
		        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		        break;
		      default:
		        break;
		    }

		vTaskDelayUntil(&time_init, BMS_MAIN_RATE);
	}
}

