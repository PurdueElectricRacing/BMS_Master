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

//global variables
volatile WatchDawg_t wdawg[NUM_SLAVES];

/***************************************************************************
*
*     Function Information
*
*     Name of Function: HAL_CAN_RxFifo0MsgPendingCallback
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. CAN_HandleTypeDef *hcan      Can Handle
*
*      Global Dependents:
*       1. None
*
*     Function Description: After a message has been received add it to the
*     rx can queue and move on with life.
*
***************************************************************************/
void HAL_CAN3_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
	CanRxMsgTypeDef rx;
  TickType_t temp;
  CAN_RxHeaderTypeDef header;
  HAL_CAN_GetRxMessage(hcan, 0, &header, rx.Data);
  rx.DLC = header.DLC;
  rx.StdId = header.StdId;
  xQueueSendFromISR(bms.q_rx_bmscan, &rx, NULL);

  //master watchdawg task
  //first data byte always corresponds to the slave ID
  if (xSemaphoreTakeFromISR(wdawg[rx.Data[0]].master_sem, NULL) == pdPASS) {
    //semaphore successfully taken
    temp = wdawg[rx.Data[0]].new_msg;
    wdawg[rx.Data[0]].new_msg = xTaskGetTickCountFromISR();
    wdawg[rx.Data[0]].last_msg = temp;
    xSemaphoreGiveFromISR(wdawg[rx.Data[0]].master_sem, NULL); //give the sem back
  }
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: HAL_CAN_RxFifo1MsgPendingCallback
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: None
*
*     Parameters (list data type, name, and comment one per line):
*       1. CAN_HandleTypeDef *hcan      Can Handle
*
*      Global Dependents:
*       1. None
*
*     Function Description: After a message has been received add it to the
*     rx can queue and move on with life.
*
***************************************************************************/
void HAL_CAN3_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
	CanRxMsgTypeDef rx;
  TickType_t temp;
  CAN_RxHeaderTypeDef header;
  HAL_CAN_GetRxMessage(hcan, 0, &header, rx.Data);
  rx.DLC = header.DLC;
  rx.StdId = header.StdId;
  xQueueSendFromISR(bms.q_rx_bmscan, &rx, NULL);

  //master watchdawg task
  //first data byte always corresponds to the slave ID
  if (xSemaphoreTakeFromISR(wdawg[rx.Data[0]].master_sem, NULL) == pdPASS) {
    //semaphore successfully taken
    temp = wdawg[rx.Data[0]].new_msg;
    wdawg[rx.Data[0]].new_msg = xTaskGetTickCountFromISR();
    wdawg[rx.Data[0]].last_msg = temp;
    xSemaphoreGiveFromISR(wdawg[rx.Data[0]].master_sem, NULL); //give the sem back
  }
}

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
	//filter 0
  CAN_FilterTypeDef FilterConf;
  FilterConf.FilterIdHigh =         ID_SLAVE_ACK << 5;
  FilterConf.FilterIdLow =          ID_SLAVE_FAULT << 5;
  FilterConf.FilterMaskIdHigh =     ID_SLAVE_TEMP << 5;
  FilterConf.FilterMaskIdLow =      ID_SLAVE_VOLT << 5;
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
  FilterConf.FilterBank = 0;
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
***************************************************************************/
void task_Slave_WDawg() {
  TickType_t time_init = 0;
  uint8_t i = 0;
  //init watch dawg
  for (i = 0; i < NUM_SLAVES; i ++) {
    xSemaphoreGive(wdawg[i].master_sem); //allows it to be taken
    wdawg[i].last_msg = 0;
    wdawg[i].new_msg = 0;
  }

  i = 0;

  while (1) {
    time_init = xTaskGetTickCount();
    i =  (i + 1) % NUM_SLAVES;
    if (xSemaphoreTake(wdawg[i].master_sem, TIMEOUT) == pdPASS) {
    	//check if past the timeout
  		if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
				if (wdawg[i].new_msg - wdawg[i].last_msg > WDAWG_TIMEOUT) {
				//this slave is now not detected
					bms.fault.slave[i].connected = FAULTED;
				} else {
					bms.fault.slave[i].connected = NORMAL;
				}
				xSemaphoreGive(bms.fault.sem);
  		}
    	xSemaphoreGive(wdawg[i].master_sem);
    } else {
    	//semaphore not acquired
    }
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
      while (!HAL_CAN_GetTxMailboxesFreeLevel(periph.bmscan)); // while mailboxes not free
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
  uint8_t i = 0;
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
					bms.fault.slave[rx_can.Data[0]].volt_sens = (fault_t) bit_extract(FAULT_MODL_VOLT_MASK, FAULT_MODL_VOLT_SHIFT, rx_can.Data[1]);
					bms.fault.slave[rx_can.Data[0]].temp_sens = (fault_t) bit_extract(FAULT_MODL_TEMP_MASK, FAULT_MODL_TEMP_SHIFT, rx_can.Data[1]);
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
*     array accordingly and does safety checks accordingly
***************************************************************************/
success_t process_temp(CanRxMsgTypeDef* rx) {
	success_t status = SUCCESSFUL;
	uint8_t loc = rx->Data[1] * 3; //beginning spot in the array
	uint8_t slave = rx->Data[0];
	fault_t overtemp = NORMAL;
	fault_t undertemp = NORMAL;
	flag_t flag = DEASSERTED;

	int16_t temp1 = byte_combine((uint16_t) rx->Data[2], (uint16_t) rx->Data[3]);
	int16_t temp2 = byte_combine((uint16_t) rx->Data[4], (uint16_t) rx->Data[5]);
	int16_t temp3 = byte_combine((uint16_t) rx->Data[6], (uint16_t) rx->Data[7]);

	//safety check
	//overtemp check
	if (temp1 > bms.params.temp_high_lim ||
			temp2 > bms.params.temp_high_lim ||
			temp3 > bms.params.temp_high_lim) {
		overtemp = FAULTED;
		flag = ASSERTED;
	}
	//undertemp check
	if (temp1 < bms.params.temp_low_lim ||
			temp2 < bms.params.temp_low_lim ||
			temp3 < bms.params.temp_low_lim) {
		undertemp = FAULTED;
		flag = ASSERTED;
	}

	if (flag == ASSERTED) {
		if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
			bms.fault.overtemp = overtemp;
			bms.fault.undertemp = undertemp;
			xSemaphoreGive(bms.fault.sem);
		} else {
			status = FAILURE;
		}
	}

	//update the table
	if (xSemaphoreTake(bms.temp.sem, TIMEOUT) == pdPASS) {
		bms.temp.data[slave][loc++] = temp1;
		bms.temp.data[slave][loc++] = temp2;
		bms.temp.data[slave][loc] = temp3;
		xSemaphoreGive(bms.temp.sem);
	} else {
		status = FAILURE;
	}

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
*     array accordingly and does safety checks accordingly
***************************************************************************/
success_t process_volt(CanRxMsgTypeDef* rx) {
	success_t status = SUCCESSFUL;
	uint8_t loc = rx->Data[1] * 3; //beginning spot in the array
	uint8_t slave = rx->Data[0];
	fault_t overvolt = NORMAL;
	fault_t undervolt = NORMAL;
	flag_t flag = DEASSERTED;

	uint16_t volt1 = byte_combine((uint16_t) rx->Data[2], (uint16_t) rx->Data[3]);
	uint16_t volt2 = byte_combine((uint16_t) rx->Data[4], (uint16_t) rx->Data[5]);
	uint16_t volt3 = byte_combine((uint16_t) rx->Data[6], (uint16_t) rx->Data[7]);

	//safety check
	//overvolt check
	if (volt1 > bms.params.volt_high_lim ||
			volt2 > bms.params.volt_high_lim ||
			volt3 > bms.params.volt_high_lim) {
		overvolt = FAULTED;
		flag = ASSERTED;
	}
	//undervolt check
	if (volt1 < bms.params.volt_low_lim ||
			volt2 < bms.params.volt_low_lim ||
			volt3 < bms.params.volt_low_lim) {
		undervolt = FAULTED;
		flag = ASSERTED;
	}

	if (flag == ASSERTED) {
		if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
			bms.fault.overvolt = overvolt;
			bms.fault.undervolt = undervolt;
			xSemaphoreGive(bms.fault.sem);
		} else {
			status = FAILURE;
		}
	}

	//update the table
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


