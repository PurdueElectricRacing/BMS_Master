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
  		if (xSemaphoreTake(bms.fault.error_sem, TIMEOUT) == pdPASS) {
				if (wdawg[i].new_msg - wdawg[i].last_msg > WDAWG_TIMEOUT) {
				//this slave is now not detected
					bms.fault.slave[i].connected = FAULTED;
				} else {
					bms.fault.slave[i].connected = NORMAL;
				}
				xSemaphoreGive(bms.fault.error_sem);
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
  TickType_t time_init = 0;
  while (1) {
    time_init = xTaskGetTickCount();

    if (xQueuePeek(bms.q_rx_bmscan, &rx_can, TIMEOUT) == pdTRUE) {
      xQueueReceive(bms.q_rx_bmscan, &rx_can, TIMEOUT);

      switch (rx_can.StdId) {
      case ID_SLAVE_ACK:
      	break;
      case ID_SLAVE_FAULT:
      	break;
      case ID_SLAVE_TEMP:
      	break;
      case ID_SLAVE_VOLT:

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

