/***************************************************************************
 *
 *     File Information
 *
 *     Name of File: can.c
 *
 *     Authors (Include Email):
 *       1. Matthew Flanagan       matthewdavidflanagan@outlook.com
 *
 *     File dependents: (header files, flow charts, referenced documentation)
 *       1. can.h
 *
 *     File Description: This manages all of the can being sent for the slave bms board
 *
 ***************************************************************************/

#include "bms_can.h"

Success_t process_temp(CanRxMsgTypeDef* rx);
Success_t process_volt(CanRxMsgTypeDef* rx);

//global variables

/***************************************************************************
 *
 *     Function Information
 *
 *     Name of Function: can_filter_init
 *
 *     Programmer's Name: Matt Flanagan
 *
 *     Function Return Type: None
 *
 *     Parameters (list data type, name, and comment one per line):
 *       1. CAN_HandleTypeDef* hcan        Can Handle
 *
 *      Global Dependents:
 *       1. None
 *
 *     Function Description: Sets the can filter to only take Messages from BMS master.
 *     Only uses FIFO0. If more messages need to be read change FilterMaskIdHigh
 *     and FilterMaskIdLow.
 *
 ***************************************************************************/
void bms_can_filter_init(CAN_HandleTypeDef* hcan) {
	// Filter to bank 0
	CAN_FilterTypeDef FilterConf;
	FilterConf.FilterIdHigh = ID_SLAVE_ACK << 5;
	FilterConf.FilterIdLow = ID_SLAVE_FAULT << 5;
	FilterConf.FilterMaskIdHigh = ID_SLAVE_TEMP << 5;
	FilterConf.FilterMaskIdLow = ID_SLAVE_VOLT << 5;
	FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
	FilterConf.FilterBank = 0;
	FilterConf.FilterMode = CAN_FILTERMODE_IDMASK;
	FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
	FilterConf.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hcan, &FilterConf);

	// Filter to bank 1
	FilterConf.FilterIdHigh = ID_SLAVE_TEMP << 5;
	FilterConf.FilterIdLow = ID_SLAVE_VOLT << 5;
	FilterConf.FilterMaskIdHigh = 0;
	FilterConf.FilterMaskIdLow = 0;
	FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
	FilterConf.FilterBank = 1;
	FilterConf.FilterMode = CAN_FILTERMODE_IDMASK;
	FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
	FilterConf.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hcan, &FilterConf);

}

/***************************************************************************
 *
 *     Function Information
 *
 *     Name of Function: master_watchDawg
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
 *     Function Description: Monitors can traffic to ensure Master is still in
 *     comms, if it doesn't receive a message from Master within x time then
 *     enter sleep mode
 *     //TODO: check for overflows
 ***************************************************************************/
void task_Slave_WDawg() {
	TickType_t time_init = 0;
	TickType_t curr_time = 0;
	uint8_t i = 0;
	CanTxMsgTypeDef msg;
	msg.IDE = CAN_ID_STD;
	msg.RTR = CAN_RTR_DATA;
	msg.DLC = WDAWG_LENGTH;
	msg.StdId = ID_WDAWG;
	msg.Data[0] = 1;
	//init watch dawg
	i = 0;

	while (1) {
		time_init = xTaskGetTickCount();
		i = (i + 1) % NUM_SLAVES;
		if (xSemaphoreTake(wdawg[i].sem, TIMEOUT) == pdPASS) {
			//check if past the timeout
			if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
				curr_time = xTaskGetTickCount();
				if ((curr_time - wdawg[i].last_msg > WDAWG_TIMEOUT)
						|| wdawg[i].last_msg == NO_MESSAGES_RECV) { //if no messages received don't check
					//this slave is now not detected
					bms.fault.slave[i].connected = FAULTED;
				} else {
					bms.fault.slave[i].connected = NORMAL;
				}
				xSemaphoreGive(bms.fault.sem);
			}
			xSemaphoreGive(wdawg[i].sem);
		} else {
			//semaphore not acquired
		}
//    if (bms.state == NORMAL_OP) {
		xQueueSendToBack(bms.q_tx_bmscan, &msg, TIMEOUT);
//    }

		vTaskDelayUntil(&time_init, WDAWG_RATE);
	}
}

/***************************************************************************
 *
 *     Function Information
 *
 *     Name of Function: task_txCan
 *
 *     Programmer's Name: Matt Flanagan
 *
 *     Function Return Type: None
 *
 *     Parameters (list data type, name, and comment one per line):
 *       1. None
 *
 *      Global Dependents:
 *       1. Can queue and such
 *
 *     Function Description: Task that runs every CAN_TX_RATE and polls for can
 *     messages to arrive to send them out to the bms master.
 *
 ***************************************************************************/
void task_txBmsCan() {
  CanTxMsgTypeDef tx;
  TickType_t time_init = 0;
  while (1) {
    time_init = xTaskGetTickCount();
    //check if this task is triggered
    if (xQueuePeek(bms.q_tx_bmscan, &tx, TIMEOUT) == pdTRUE) {
      xQueueReceive(bms.q_tx_bmscan, &tx, TIMEOUT);  //actually take item out of queue
      CAN_TxHeaderTypeDef header;
      header.DLC = tx.DLC;
      header.IDE = tx.IDE;
      header.RTR = tx.RTR;
      header.StdId = tx.StdId;
      header.TransmitGlobalTime = DISABLE;
      uint32_t mailbox;

      //send the message

      while (!HAL_CAN_GetTxMailboxesFreeLevel(periph.bmscan));

      // while mailboxes not free
      HAL_CAN_AddTxMessage(periph.bmscan, &header, tx.Data, &mailbox);
    }
    vTaskDelayUntil(&time_init, CAN_TX_RATE);
  }
}

/***************************************************************************
 *
 *     Function Information
 *
 *     Name of Function: task_CanProcess
 *
 *     Programmer's Name: Matt Flanagan
 *
 *     Function Return Type: None
 *
 *     Parameters (list data type, name, and comment one per line):
 *       1. None
 *
 *      Global Dependents:
 *       1. Can queue and such
 *
 *     Function Description: Processes all of the new messages that have been
 *     received via can.
 ***************************************************************************/
void task_BmsCanProcess() {
	CanRxMsgTypeDef rx_can;
	TickType_t time_init = 0;
	while (1) {
		time_init = xTaskGetTickCount();

		if (xQueuePeek(bms.q_rx_bmscan, &rx_can, TIMEOUT) == pdTRUE) {
			xQueueReceive(bms.q_rx_bmscan, &rx_can, TIMEOUT);

			switch (rx_can.StdId) {
			case ID_SLAVE_ACK:
				//Don't need to do anything Master Watch dawg task takes care of it
				break;
			case ID_SLAVE_FAULT:
				if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
					//connected is not relevant because Master Watch dawg is looking for that
					bms.fault.slave[rx_can.Data[0]].volt_sens =
							(fault_t) bit_extract(FAULT_MODL_VOLT_MASK,
									FAULT_MODL_VOLT_SHIFT, rx_can.Data[1]);
					bms.fault.slave[rx_can.Data[0]].temp_sens =
							(fault_t) bit_extract(FAULT_MODL_TEMP_MASK,
									FAULT_MODL_TEMP_SHIFT, rx_can.Data[1]);
					xSemaphoreGive(bms.fault.sem);
				}
				break;
			case ID_SLAVE_TEMP:
				process_temp(&rx_can);
				break;
			case ID_SLAVE_VOLT:
				process_volt(&rx_can);
				break;
			default:
				//invalid can message TODO: handle this
				break;
			}
		}

		vTaskDelayUntil(&time_init, CAN_RX_RATE);
	}
}

/***************************************************************************
 *
 *     Function Information
 *
 *     Name of Function: process_temp
 *
 *     Programmer's Name: Matt Flanagan
 *
 *     Function Return Type: None
 *
 *     Parameters (list data type, name, and comment one per line):
 *       1. None
 *
 *      Global Dependents:
 *       1. Bms.temp array
 *       2. bms.fault
 *
 *     Function Description: takes the newest received data and updates the temp
 *     array accordingly and does safety checks accordingly. This will take in 17 values
 ***************************************************************************/
Success_t process_temp(CanRxMsgTypeDef* rx) {
	Success_t status = SUCCESSFUL;
	uint8_t loc = rx->Data[1] * 3; //beginning spot in the array
	uint8_t slave = rx->Data[0];

	int16_t temp1 = byte_combine((uint16_t ) rx->Data[2],
			(uint16_t ) rx->Data[3]);
	int16_t temp2 = byte_combine((uint16_t ) rx->Data[4],
			(uint16_t ) rx->Data[5]);
//  int16_t temp3 = byte_combine((uint16_t) rx->Data[6], (uint16_t) rx->Data[7]);

	//update the table //todo check to make sure we don't go over array bounds...
//  if (xSemaphoreTake(bms.temp.sem, TIMEOUT) == pdPASS) {
	bms.temp.data[slave][loc++] = temp1;
	bms.temp.data[slave][loc++] = temp2;
//    bms.temp.data[slave][loc] = temp3;
//    xSemaphoreGive(bms.temp.sem);
//  } else {
//    status = FAILURE;
//  }

	return status;
}

/***************************************************************************
 *
 *     Function Information
 *
 *     Name of Function: process_volt
 *
 *     Programmer's Name: Matt Flanagan
 *
 *     Function Return Type: None
 *
 *     Parameters (list data type, name, and comment one per line):
 *       1. None
 *
 *      Global Dependents:
 *       1. Bms.volt array
 *       2. bms.fault
 *
 *     Function Description: takes the newest received data and updates the volt
 *     array accordingly and does safety checks accordingly. This will have 21 values
 ***************************************************************************/
Success_t process_volt(CanRxMsgTypeDef* rx) {
	Success_t status = SUCCESSFUL;
	uint8_t loc = rx->Data[1] * 3; //beginning spot in the array
	uint8_t slave = rx->Data[0];

	uint16_t volt1 = byte_combine((uint16_t ) rx->Data[2],
			(uint16_t ) rx->Data[3]);
	uint16_t volt2 = byte_combine((uint16_t ) rx->Data[4],
			(uint16_t ) rx->Data[5]);
	uint16_t volt3 = byte_combine((uint16_t ) rx->Data[6],
			(uint16_t ) rx->Data[7]);

	//update the table //todo check to make sure we don't go over array bounds...
	//TODO: update this to have 21 values
	if (xSemaphoreTake(bms.vtaps.sem, TIMEOUT) == pdPASS) {
		bms.vtaps.data[slave][loc++] = volt1;
		bms.vtaps.data[slave][loc++] = volt2;
		bms.vtaps.data[slave][loc] = volt3;
		xSemaphoreGive(bms.vtaps.sem);
	} else {
		status = FAILURE;
	}

	return status;
}
