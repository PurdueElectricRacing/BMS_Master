/*
 * bms.c
 *
 *  Created on: Feb 11, 2019
 *      Author: Matt Flanagan
 */
#include "bms.h"

void HAL_GPIO_EXTI_Callback(uint16_t pin) {

	switch (pin) {
	case POWER_LOSS_PIN: //TODO confirm this is correct
		if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
			bms.state = SHUTDOWN;
			xSemaphoreGive(bms.state_sem); //release sem
		}
		break;
	default:
		break;
	}
}

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
*     Name of Function: task_error_check
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. None
*
*      Global Dependents:
*       1. bms.fault
*
*     Function Description: Checks each error case at  ERROR_CHECK_RATE and
*     will fault the BMS if an error has been detected.
*
***************************************************************************/
void task_error_check() {
  TickType_t time_init = 0;
  uint8_t i = 0;
  fault_t fault = NORMAL;
  while (1) {
    time_init = xTaskGetTickCount();

    if (bms.fault.charg_en == FAULTED ||
    		bms.fault.discharg_en == FAULTED ||
				bms.fault.overtemp == FAULTED ||
				bms.fault.undertemp == FAULTED ||
				bms.fault.overvolt == FAULTED ||
				bms.fault.undervolt == FAULTED) {
    	fault = FAULTED;
    }

		for (i = 0; i < NUM_SLAVES; i++) {
				if (bms.fault.slave[i].connected == FAULTED ||
						bms.fault.slave[i].temp_sens == FAULTED ||
						bms.fault.slave[i].volt_sens == FAULTED) {
					fault = FAULTED;
				}
		}

		if (fault == FAULTED) {
			if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
				bms.state = ERROR_BMS;
				xSemaphoreGive(bms.state_sem); //release sem
			}
		}

    vTaskDelayUntil(&time_init, ERROR_CHECK_RATE);
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
  bms.q_rx_dcan		= xQueueCreate(DCAN_RX_Q_SIZE, sizeof(CanRxMsgTypeDef));
  bms.q_tx_dcan		= xQueueCreate(DCAN_RX_Q_SIZE, sizeof(CanRxMsgTypeDef));
  bms.q_rx_dcan		= xQueueCreate(DCAN_RX_Q_SIZE, sizeof(CanRxMsgTypeDef));
	bms.q_tx_dcan		= xQueueCreate(DCAN_RX_Q_SIZE, sizeof(CanRxMsgTypeDef));

  //start tasks
	xTaskCreate(task_Slave_WDawg, "Master WDawg", WDAWG_STACK_SIZE, NULL, WDAWG_PRIORITY, NULL);
  xTaskCreate(task_txBmsCan, "Transmit BmsCan", BMSCAN_TX_STACK_SIZE, NULL, BMSCAN_TX_PRIORITY, NULL);
  xTaskCreate(task_BmsCanProcess, "BMS Can RX", BMSCAN_RX_STACK_SIZE, NULL, BMSCAN_TX_PRIORITY, NULL);
  xTaskCreate(task_bms_main, "Main Task", BMS_MAIN_STACK_SIZE, NULL, BMS_MAIN_PRIORITY, NULL);
  xTaskCreate(task_heartbeat, "Heartbeat", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_PRIORITY, NULL);
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
	uint8_t x = 0;
  bms.state_sem = xSemaphoreCreateBinary();

  //TODO read these limits off of the SD Card maybe?
  bms.params.charg_lim = LIMIT_CHARG;
  bms.params.discharg_lim = LIMIT_DISCHARG;
  bms.params.temp_high_lim = LIMIT_TEMP_HIGH;
  bms.params.temp_low_lim = LIMIT_TEMP_LOW;
  bms.params.volt_high_lim = LIMIT_VOLT_HIGH;
  bms.params.volt_low_lim = LIMIT_VOLT_LOW;
  bms.params.ir_msg_en = DEASSERTED;
  bms.params.ocv_msg_en = DEASSERTED;
  bms.params.temp_msg_en = DEASSERTED;
  bms.params.volt_msg_en = DEASSERTED;
  bms.params.macro_msg_en = ASSERTED;
  bms.params.sem = xSemaphoreCreateBinary();

  bms.fault.charg_en = NORMAL;
  bms.fault.discharg_en = NORMAL;
  bms.fault.sem = xSemaphoreCreateBinary();
  bms.fault.overtemp = NORMAL;
  bms.fault.undertemp = NORMAL;
  bms.fault.overvolt = NORMAL;
  bms.fault.undervolt = NORMAL;

  for (i = 0; i < NUM_SLAVES; i++) {
    bms.fault.slave[i].connected = NORMAL;
    bms.fault.slave[i].temp_sens = NORMAL;
    bms.fault.slave[i].volt_sens = NORMAL;
    bms.vtaps.sem = xSemaphoreCreateBinary();
    bms.temp.sem = xSemaphoreCreateBinary();
    //initialize all vtap data
    for (x = 0; x < NUM_VTAPS; x ++) {
    	bms.vtaps.data[i][x] = 0;
    }
    for (x= 0; x < NUM_TEMP; x ++) {
    	bms.temp.data[i][x] = 0;
    }
  }

  bms.state = INIT;

  xSemaphoreGive(bms.state_sem);
  xSemaphoreGive(bms.fault.sem);
  xSemaphoreGive(bms.vtaps.sem);
  xSemaphoreGive(bms.temp.sem);
  xSemaphoreGive(bms.params.sem);

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
	uint8_t i = 0;
	TickType_t time_init = 0;
	while (1) {
		time_init = xTaskGetTickCount();
		i++;
		switch (bms.state) {
		      case INIT:
		      	//TODO: do self checks
		      	//TODO: confirm current sense
		      	//TODO: confirm SD card connection

						if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
							bms.state = BMS_CONNECT;
							xSemaphoreGive(bms.state_sem); //release sem
						}
		        break;
		      case BMS_CONNECT:
		      	//establish connections with all slaves
		      	//send wakeup signal and poll until all slaves are connected
		      	power_cmd_slaves(POWER_ON);
		      	while (slaves_not_connected()) {
		      		vTaskDelay(DELAY_SLAVE_CON);
		      	}
		      	if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
							bms.state = NORMAL_OP;
							xSemaphoreGive(bms.state_sem); //release sem
						}
		      	break;
		      case NORMAL_OP:
		      	//TODO: start normal op tasks (get handles to all)
		        //TODO: read from all of the sensors
		        //TODO: manage passive balancing if necessary
		        break;
		      case ERROR_BMS:
		      	//TODO: kill all non critical tasks
		      	HAL_GPIO_WritePin(SDC_BMS_FAULT_GPIO_Port, SDC_BMS_FAULT_Pin, GPIO_PIN_SET); //open SDC
		        send_faults();
		        if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
		          bms.state = SHUTDOWN;
		          xSemaphoreGive(bms.state_sem); //release sem
		        }
		        break;
		      case SHUTDOWN:
		      	//TODO: kill all non critical tasks
		      	power_cmd_slaves(POWER_OFF);
		      	//TODO: finish all sd card writing
		      	if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
							bms.state = SOFT_RESET;
							xSemaphoreGive(bms.state_sem); //release sem
						}
		        break;
		      case SOFT_RESET:
		      	if (bms.fault.clear == ASSERTED) {
		      		if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
		      			clear_faults();
		      			xSemaphoreGive(bms.fault.sem);
								if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
									bms.state = INIT;
									bms.fault.clear = DEASSERTED;
									xSemaphoreGive(bms.state_sem); //release sem
								}
		      		}
		      	} else {
		      		vTaskDelay(DELAY_RESET);
		      	}
		      	break;
		      default:
		      	//never can get here
		        break;
		    }

		if (i % 100 == 0) {
			HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
		}
		vTaskDelayUntil(&time_init, BMS_MAIN_RATE);
	}
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: power_cmd_slaves
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: Success status
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*       1. q_tx_bmscan
*
*     Function Description: sends a wakeup message to the slaves
***************************************************************************/
success_t power_cmd_slaves(powercmd_t poweron) {
	CanTxMsgTypeDef msg;
	msg.IDE = CAN_ID_STD;
	msg.RTR = CAN_RTR_DATA;
	msg.DLC = 1;
	msg.StdId = ID_BMS_WAKEUP;
	msg.Data[0] = poweron;

	xQueueSendToBack(bms.q_tx_bmscan, &msg, 100);
	return SUCCESSFUL;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: slaves_not_connected
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: Success status
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*       1.
*
*     Function Description: sends a wakeup message to the slaves
***************************************************************************/
success_t slaves_not_connected() {
	uint8_t i = 0;
	success_t success = SUCCESSFUL;
	for (i = 0; i < NUM_SLAVES; i++) {
		if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdTRUE) {
			if(bms.fault.slave[i].connected == FAULTED) {
				success = FAILURE;
			}
			xSemaphoreGive(bms.fault.sem);
		}
	}

	return success;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: clear_faults
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: Success status
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*       1.
*
*     Function Description: clears all pending faults in the BMS
***************************************************************************/
success_t clear_faults() {
	uint8_t i = 0;

	bms.fault.charg_en = NORMAL;
	bms.fault.discharg_en = NORMAL;
	bms.fault.overtemp = NORMAL;
	bms.fault.undertemp = NORMAL;
	bms.fault.overvolt = NORMAL;
	bms.fault.undervolt = NORMAL;

	for (i = 0; i < NUM_SLAVES; i++) {
	    bms.fault.slave[i].connected = NORMAL;
	    bms.fault.slave[i].temp_sens = NORMAL;
	    bms.fault.slave[i].volt_sens = NORMAL;
	}

	return SUCCESSFUL;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: wakeup_slaves
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: Success status
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*       1. q_tx_bmscan
*
*     Function Description: sends fault code to the GUI see CAN msg docs for
*     more info on what each bit is representing
***************************************************************************/
success_t send_faults() {
	CanTxMsgTypeDef msg;
	uint8_t i = 0;
	msg.IDE = CAN_ID_STD;
	msg.RTR = CAN_RTR_DATA;
	msg.DLC = NUM_SLAVES + 1; //one for the macro faults
	msg.StdId = ID_MASTER_ERROR_MSG;

	//macro faults
	msg.Data[0] = bitwise_or(FAULT_CHARGE_EN_SHIFT, FAULT_CHARGE_EN_MASK, bms.fault.charg_en);
	msg.Data[0] |= bitwise_or(FAULT_DISCHARGE_EN_SHIFT, FAULT_DISCHARGE_EN_MASK, bms.fault.discharg_en);
	msg.Data[0] |= bitwise_or(FAULT_OVERTEMP_SHIFT, FAULT_OVERTEMP_MASK, bms.fault.overtemp);
	msg.Data[0] |= bitwise_or(FAULT_OVERVOLT_SHIFT, FAULT_OVERVOLT_MASK, bms.fault.overvolt);
	msg.Data[0] |= bitwise_or(FAULT_UNDERVOLT_SHIFT, FAULT_UNDERVOLT_MASK, bms.fault.undervolt);
	msg.Data[0] |= bitwise_or(FAULT_UNDERTEMP_SHIFT, FAULT_UNDERTEMP_MASK, bms.fault.undertemp);

	//module specific faults
	for (i = 0; i < NUM_SLAVES; i++) {
		//low module
		msg.Data[i] = bitwise_or(FAULT_MODL_CON_SHIFT, FAULT_MODL_CON_MASK, bms.fault.slave[i].connected);
		msg.Data[i] |= bitwise_or(FAULT_MODL_TEMP_SHIFT, FAULT_MODL_TEMP_MASK, bms.fault.slave[i].temp_sens);
		msg.Data[i] |= bitwise_or(FAULT_MODL_VOLT_SHIFT, FAULT_MODL_VOLT_MASK, bms.fault.slave[i].volt_sens);

		//high module
		msg.Data[i] |= bitwise_or(FAULT_MODH_CON_SHIFT, FAULT_MODH_CON_MASK, bms.fault.slave[i].connected);
		msg.Data[i] |= bitwise_or(FAULT_MODH_TEMP_SHIFT, FAULT_MODH_TEMP_MASK, bms.fault.slave[i].temp_sens);
		msg.Data[i] |= bitwise_or(FAULT_MODH_VOLT_SHIFT, FAULT_MODH_VOLT_MASK, bms.fault.slave[i].volt_sens);
	}

	xQueueSendToBack(bms.q_tx_dcan, &msg, 100);
	return SUCCESSFUL;
}








