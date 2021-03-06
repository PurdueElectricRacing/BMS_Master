/***************************************************************************
*
*     File Information
*
*     Name of File: dcan.c
*
*     Authors (Include Email):
*       1. Matthew Flanagan       matthewdavidflanagan@outlook.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. can.h
*
*     File Description: This manages all of the can being sent on DCAN which includes GUI
*
***************************************************************************/

#include "dcan.h"

Success_t process_gui_cmd(CanRxMsgTypeDef* rx_can);
Success_t process_gui_param_set(CanRxMsgTypeDef* rx_can);
Success_t process_gui_param_req(CanRxMsgTypeDef* rx_can);
Success_t send_volt_msg();
Success_t send_temp_msg();
Success_t send_ocv_msg();
Success_t send_ir_msg();
Success_t send_macro_msg();
Success_t send_generic_msg(uint16_t items, dcan_broadcast_t msg_type);

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
void task_txDcan() {
  CanTxMsgTypeDef tx;
  TickType_t time_init = 0;
  while (1) {
    time_init = xTaskGetTickCount();
    //check if this task is triggered
    if (xQueuePeek(bms.q_tx_dcan, &tx, TIMEOUT) == pdTRUE) {
      xQueueReceive(bms.q_tx_dcan, &tx, TIMEOUT);  //actually take item out of queue
      CAN_TxHeaderTypeDef header;
      header.DLC = tx.DLC;
      header.IDE = tx.IDE;
      header.RTR = tx.RTR;
      header.StdId = tx.StdId;
      header.TransmitGlobalTime = DISABLE;
      uint32_t mailbox;
      //send the message
      while (!HAL_CAN_GetTxMailboxesFreeLevel(periph.dcan)); // while mailboxes not free
      HAL_CAN_AddTxMessage(periph.dcan, &header, tx.Data, &mailbox);
    }
    vTaskDelayUntil(&time_init, DCAN_TX_RATE);
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
void task_DcanProcess() {
  CanRxMsgTypeDef rx_can;
  TickType_t time_init = 0;
  while (1) {
    time_init = xTaskGetTickCount();
    
    if (xQueuePeek(bms.q_rx_dcan, &rx_can, TIMEOUT) == pdTRUE) {
      xQueueReceive(bms.q_rx_dcan, &rx_can, TIMEOUT);
      
      switch (rx_can.StdId) {
        case ID_GUI_CMD:
          process_gui_cmd(&rx_can);
          break;
        case ID_GUI_BMS_RESET:
          if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdTRUE) {
            bms.fault.clear = ASSERTED;
            xSemaphoreGive(bms.fault.sem);
          }
          break;
        case ID_GUI_PARAM_SET:
          process_gui_param_set(&rx_can);
          break;
        case ID_GUI_PARAM_REQ:
          process_gui_param_req(&rx_can);
          break;
      }
    }
    
    vTaskDelayUntil(&time_init, DCAN_RX_RATE);
  }
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: task_broadcast
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
*     Function Description: brodcasts msg's at the defined rate and if they are
*     enabled
***************************************************************************/
void task_broadcast() {
  TickType_t time_init = 0;
  uint16_t i = 0;
  while (1) {
    time_init = xTaskGetTickCount();
//    if (bms.state == NORMAL_OP)
    {
      if (bms.params.volt_msg_en == ASSERTED) {
        if (execute_broadcast(bms.params.volt_msg_rate, i)) {
          send_volt_msg();
        }
        vTaskDelay(BROADCAST_DELAY);
      }
      if (bms.params.temp_msg_en == ASSERTED) {
        if (execute_broadcast(bms.params.temp_msg_rate, i)) {
          send_temp_msg();
        }
        vTaskDelay(BROADCAST_DELAY);
      }
      if (bms.params.ocv_msg_en == ASSERTED) {
        if (execute_broadcast(bms.params.ocv_msg_rate, i)) {
          send_ocv_msg();
        }
        vTaskDelay(BROADCAST_DELAY);
      }
      if (bms.params.ir_msg_en == ASSERTED) {
        if (execute_broadcast(bms.params.ir_msg_rate, i)) {
          send_ir_msg();
        }
        vTaskDelay(BROADCAST_DELAY);
      }
      if (bms.params.macro_msg_en == ASSERTED) {
        if (execute_broadcast(bms.params.macro_msg_rate, i)) {
          send_macro_msg();
        }
      }
    }
    
    i++;
    vTaskDelayUntil(&time_init, BROADCAST_RATE);
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
void dcan_filter_init(CAN_HandleTypeDef* hcan) {
  CAN_FilterTypeDef FilterConf;
  FilterConf.FilterIdHigh =         ID_GUI_CMD << 5; //1
  FilterConf.FilterIdLow =          ID_GUI_BMS_RESET << 5; //2
  FilterConf.FilterMaskIdHigh =     ID_GUI_PARAM_SET << 5; //3
  FilterConf.FilterMaskIdLow =      ID_GUI_PARAM_REQ << 5; //4
  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
  FilterConf.FilterBank = 0;
  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  FilterConf.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan, &FilterConf);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: process_gui_cmd
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: Success_t
*
*     Parameters (list data type, name, and comment one per line):
*       1. CanRxMsgTypeDef* rx_can
*
*      Global Dependents:
*       1. bms.params
*       2. sd card
*
*     Function Description: processes a gui cmd msg. SD card operations and
*     configs broadcast paramaters for msg's on dcan
*
***************************************************************************/
Success_t process_gui_cmd(CanRxMsgTypeDef* rx_can) {
  Success_t status = SUCCESSFUL;
  
  switch (rx_can->Data[0]) {
    case LOG_DATA:
      //TODO: raymond sd card read/send enable
      break;
    case DELETE:
      //TODO: raymond delete sd card data
      break;
    case CONFIGURE:
      //configure broadcast operations
      if (xSemaphoreTake(bms.params.sem, TIMEOUT) == pdTRUE) {
        bms.params.ir_msg_en = (flag_t) bit_extract(CONFIG_IR_MSG_MASK, CONFIG_IR_MSG_SHIFT,
                               rx_can->Data[1]);
        bms.params.ocv_msg_en = (flag_t) bit_extract(CONFIG_OCV_MSG_MASK, CONFIG_OCV_MSG_SHIFT,
                                rx_can->Data[1]);
        bms.params.temp_msg_en = (flag_t) bit_extract(CONFIG_TEMP_MSG_MASK, CONFIG_TEMP_MSG_SHIFT,
                                 rx_can->Data[1]);
        bms.params.volt_msg_en = (flag_t) bit_extract(CONFIG_VOLT_MSG_MASK, CONFIG_VOLT_MSG_SHIFT,
                                 rx_can->Data[1]);
        bms.params.macro_msg_en = (flag_t) bit_extract(CONFIG_MACRO_MSG_MASK, CONFIG_MACRO_MSG_SHIFT,
                                  rx_can->Data[1]);
        xSemaphoreGive(bms.params.sem);
      } else {
        status = FAILURE;
      }
      break;
    case NO_OPERATION:
      //self explanatory what do you expect here
      break;
  }
  
  return status;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: process_gui_param_set
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: Success_t
*
*     Parameters (list data type, name, and comment one per line):
*       1. CanRxMsgTypeDef* rx_can
*
*      Global Dependents:
*       1. bms.params
*
*     Function Description: processes a gui param config msg and sets the appropriate variable
*
***************************************************************************/
Success_t process_gui_param_set(CanRxMsgTypeDef* rx_can) {
  Success_t status = SUCCESSFUL;
  
  if (xSemaphoreTake(bms.params.sem, TIMEOUT) == pdTRUE) {
    switch (rx_can->Data[0]) {
      case TEMP_HIGH_LIMIT:
        bms.params.temp_high_lim = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case TEMP_LOW_LIMIT:
        bms.params.temp_low_lim = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case VOLT_HIGH_LIMIT:
        bms.params.volt_high_lim = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case VOLT_LOW_LIMIT:
        bms.params.volt_low_lim = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case DISCHARGE_LIMIT:
        bms.params.discharg_lim = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case CHARGE_LIMIT:
        bms.params.charg_lim = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case VOLT_MSG_RATE:
        bms.params.volt_msg_rate = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case TEMP_MSG_RATE:
        bms.params.temp_msg_rate = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case OCV_MSG_RATE:
        bms.params.ocv_msg_rate = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case IR_MSG_RATE:
        bms.params.ir_msg_rate = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case MACRO_MSG_RATE:
        bms.params.macro_msg_rate = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
      case PASSIVE_EN:
        bms.params.passive_en = byte_combine(rx_can->Data[1], rx_can->Data[2]);
        break;
    }
    xSemaphoreGive(bms.params.sem);
  } else {
    status = FAILURE;
  }
  
  return status;
}

Success_t process_gui_param_req(CanRxMsgTypeDef* rx_can) {
  Success_t status = SUCCESSFUL;
  CanTxMsgTypeDef msg;
  msg.IDE = CAN_ID_STD;
  msg.RTR = CAN_RTR_DATA;
  msg.DLC = PARAM_RES_MSG_LENGTH;
  msg.StdId = ID_MASTER_PARAM_RES;
  msg.Data[0] = rx_can->Data[0];
  switch (rx_can->Data[0]) {
    case TEMP_HIGH_LIMIT:
      msg.Data[1] = extract_MSB(bms.params.temp_high_lim);
      msg.Data[2] = extract_LSB(bms.params.temp_high_lim);
      break;
    case TEMP_LOW_LIMIT:
      msg.Data[1] = extract_MSB(bms.params.temp_low_lim);
      msg.Data[2] = extract_LSB(bms.params.temp_low_lim);
      break;
    case VOLT_HIGH_LIMIT:
      msg.Data[1] = extract_MSB(bms.params.volt_high_lim);
      msg.Data[2] = extract_LSB(bms.params.volt_high_lim);
      break;
    case VOLT_LOW_LIMIT:
      msg.Data[1] = extract_MSB(bms.params.volt_low_lim);
      msg.Data[2] = extract_LSB(bms.params.volt_low_lim);
      break;
    case DISCHARGE_LIMIT:
      msg.Data[1] = extract_MSB(bms.params.discharg_lim);
      msg.Data[2] = extract_LSB(bms.params.discharg_lim);
      break;
    case CHARGE_LIMIT:
      msg.Data[1] = extract_MSB(bms.params.charg_lim);
      msg.Data[2] = extract_LSB(bms.params.charg_lim);
      break;
    case VOLT_MSG_RATE:
      msg.Data[1] = extract_MSB(bms.params.volt_msg_rate);
      msg.Data[2] = extract_LSB(bms.params.volt_msg_rate);
      break;
    case TEMP_MSG_RATE:
      msg.Data[1] = extract_MSB(bms.params.temp_msg_rate);
      msg.Data[2] = extract_LSB(bms.params.temp_msg_rate);
      break;
    case OCV_MSG_RATE:
      msg.Data[1] = extract_MSB(bms.params.ocv_msg_rate);
      msg.Data[2] = extract_LSB(bms.params.ocv_msg_rate);
      break;
    case IR_MSG_RATE:
      msg.Data[1] = extract_MSB(bms.params.ir_msg_rate);
      msg.Data[2] = extract_LSB(bms.params.ir_msg_rate);
      break;
    case MACRO_MSG_RATE:
      msg.Data[1] = extract_MSB(bms.params.macro_msg_rate);
      msg.Data[2] = extract_LSB(bms.params.macro_msg_rate);
      break;
    case SOC_VALUE:
      msg.Data[1] = extract_MSB(bms.macros.soc);
      msg.Data[2] = 0;
      break;
    case SOH_VALUE:
      msg.Data[1] = extract_MSB(bms.macros.soh);
      msg.Data[2] = 0;
      break;
    case PACK_VOLTAGE:
      msg.Data[1] = extract_MSB(bms.macros.pack_volt);
      msg.Data[2] = extract_LSB(bms.macros.pack_volt);
      break;
    case PACK_CURRENT:
      msg.Data[1] = extract_MSB(bms.macros.pack_i.ch2_high_current);
      msg.Data[2] = extract_LSB(bms.macros.pack_i.ch2_high_current);
      break;
    case HIGH_TEMP:
      msg.Data[1] = extract_MSB(bms.macros.high_temp.val);
      msg.Data[2] = extract_LSB(bms.macros.high_temp.val);
      break;
    case BROAD_CONFIG:
      msg.Data[1] = bitwise_or(CONFIG_VOLT_MSG_SHIFT, CONFIG_VOLT_MSG_MASK, bms.params.volt_msg_en);
      msg.Data[1] |= bitwise_or(CONFIG_TEMP_MSG_SHIFT, CONFIG_TEMP_MSG_MASK, bms.params.temp_msg_en);
      msg.Data[1] |= bitwise_or(CONFIG_OCV_MSG_SHIFT, CONFIG_OCV_MSG_MASK, bms.params.ocv_msg_en);
      msg.Data[1] |= bitwise_or(CONFIG_IR_MSG_SHIFT, CONFIG_IR_MSG_MASK, bms.params.ir_msg_en);
      msg.Data[1] |= bitwise_or(CONFIG_MACRO_MSG_SHIFT, CONFIG_MACRO_MSG_MASK, bms.params.macro_msg_en);
      msg.Data[2] = 0;
      break;
    case PASSIVE_EN:
      msg.Data[1] = (uint8_t) bms.params.passive_en;
      msg.Data[2] = 0;
      break;
  }
  
  if (xQueueSendToBack(bms.q_tx_dcan, &msg, 100) != pdPASS) {
    status = FAILURE;
  }
  return status;
}

Success_t send_volt_msg() {
  Success_t status = SUCCESSFUL;
  status = send_generic_msg(NUM_VTAPS, VOLT_MSG);
  return status;
}

Success_t send_temp_msg() {
  Success_t status = SUCCESSFUL;
  status = send_generic_msg(NUM_TEMP, TEMP_MSG);
  return status;
}

Success_t send_ocv_msg() {
  Success_t status = SUCCESSFUL;
  status = send_generic_msg(NUM_VTAPS, OCV_MSG);
  return status;
}

Success_t send_ir_msg() {
  Success_t status = SUCCESSFUL;
  status = send_generic_msg(NUM_VTAPS, IR_MSG);
  return status;
}

// can msg for macro
//[SOC_MSB, SOC_LSB, PACKVOLT_MSB, PACKVOLT_LSB, PACKI_MSB, PACKI_LSB, TEMP_MSB, TEMP_LSB]
Success_t send_macro_msg() {
  Success_t status = SUCCESSFUL;
  
  CanTxMsgTypeDef msg;
  msg.IDE = CAN_ID_STD;
  msg.RTR = CAN_RTR_DATA;
  msg.DLC = MACRO_MSG_LENGTH;
  msg.StdId = ID_MASTER_MACRO_MSG;
  msg.Data[0] = bms.macros.soc;
  msg.Data[1] = (uint8_t) (bms.macros.pack_volt >> 16);
  msg.Data[2] = (uint8_t) (bms.macros.pack_volt >> 8) & 0xFF;
  msg.Data[3] = (uint8_t) bms.macros.pack_volt;
  msg.Data[4] = extract_MSB(bms.macros.pack_i.ch1_low_current);
  msg.Data[5] = extract_LSB(bms.macros.pack_i.ch1_low_current);
  msg.Data[6] = extract_MSB(bms.macros.high_temp.val);
  msg.Data[7] = extract_LSB(bms.macros.high_temp.val);
  
  if (xQueueSendToBack(bms.q_tx_dcan, &msg, 100) != pdPASS) {
    status = FAILURE;
  }
  
  return status;
}

Success_t send_generic_msg(uint16_t items, dcan_broadcast_t msg_type) {
  Success_t status = SUCCESSFUL;
  uint8_t i = 0;
  uint8_t x = 0;
  CanTxMsgTypeDef msg;
  msg.IDE = CAN_ID_STD;
  msg.RTR = CAN_RTR_DATA;
  
  switch (msg_type) {
    case VOLT_MSG:
      for (i = 0; i < NUM_SLAVES; i++) {
        for (x = 0; x < NUM_VTAPS; x = x + VALUES_PER_MSG) {
          msg.DLC = GENERIC_MSG_LENGTH;
          msg.StdId = ID_MASTER_VOLT_MSG;
          msg.Data[0] = i;  //slave id
          msg.Data[1] = x / VALUES_PER_MSG; //row
          msg.Data[2] = extract_MSB(bms.vtaps.data[i][x]);
          msg.Data[3] = extract_LSB(bms.vtaps.data[i][x]);
          if (x + 1 < NUM_VTAPS) {
            msg.Data[4] = extract_MSB(bms.vtaps.data[i][x + 1]);
            msg.Data[5] = extract_LSB(bms.vtaps.data[i][x + 1]);
          } else {
            msg.Data[4] = 0;
            msg.Data[5] = 0;
          }
          if (x + 2 < NUM_VTAPS) {
            msg.Data[6] = extract_MSB(bms.vtaps.data[i][x + 2]);
            msg.Data[7] = extract_LSB(bms.vtaps.data[i][x + 2]);
          } else {
            msg.Data[6] = 0;
            msg.Data[7] = 0;
          }
          
          if (xQueueSendToBack(bms.q_tx_dcan, &msg, 100) != pdPASS) {
            status = FAILURE;
          }
        }
      }
      break;
    case TEMP_MSG:
      for (i = 0; i < NUM_SLAVES; i++) {
        for (x = 0; x < NUM_TEMP; x = x + VALUES_PER_MSG) {
          msg.DLC = MACRO_MSG_LENGTH;
          msg.StdId = ID_MASTER_TEMP_MSG;
          msg.Data[0] = i;  //slave id
          msg.Data[1] = x / VALUES_PER_MSG; //row
          msg.Data[2] = extract_MSB(bms.temp.data[i][x]);
          msg.Data[3] = extract_LSB(bms.temp.data[i][x]);
          if (x + 1 < NUM_TEMP) {
            msg.Data[4] = extract_MSB(bms.temp.data[i][x + 1]);
            msg.Data[5] = extract_LSB(bms.temp.data[i][x + 1]);
          } else {
            msg.Data[4] = 0;
            msg.Data[5] = 0;
          }
          if (x + 2 < NUM_TEMP) {
            msg.Data[6] = extract_MSB(bms.temp.data[i][x + 2]);
            msg.Data[7] = extract_LSB(bms.temp.data[i][x + 2]);
          } else {
            msg.Data[6] = 0;
            msg.Data[7] = 0;
          }
          
          if (xQueueSendToBack(bms.q_tx_dcan, &msg, 100) != pdPASS) {
            status = FAILURE;
          }
        }
      }
      break;
    case OCV_MSG:
      for (i = 0; i < NUM_SLAVES; i++) {
        for (x = 0; x < NUM_VTAPS; x = x + VALUES_PER_MSG) {
          msg.DLC = GENERIC_MSG_LENGTH;
          msg.StdId = ID_MASTER_OCV_MSG;
          msg.Data[0] = i;  //slave id
          msg.Data[1] = x / VALUES_PER_MSG; //row
          msg.Data[2] = extract_MSB(bms.vtaps.ocv[i][x]);
          msg.Data[3] = extract_LSB(bms.vtaps.ocv[i][x]);
          if (x + 1 < NUM_VTAPS) {
            msg.Data[4] = extract_MSB(bms.vtaps.ocv[i][x + 1]);
            msg.Data[5] = extract_LSB(bms.vtaps.ocv[i][x + 1]);
          } else {
            msg.Data[4] = 0;
            msg.Data[5] = 0;
          }
          if (x + 2 < NUM_VTAPS) {
            msg.Data[6] = extract_MSB(bms.vtaps.ocv[i][x + 2]);
            msg.Data[7] = extract_LSB(bms.vtaps.ocv[i][x + 2]);
          } else {
            msg.Data[6] = 0;
            msg.Data[7] = 0;
          }
          
          if (xQueueSendToBack(bms.q_tx_dcan, &msg, 100) != pdPASS) {
            status = FAILURE;
          }
        }
      }
      break;
    case IR_MSG:
      for (i = 0; i < NUM_SLAVES; i++) {
        for (x = 0; x < NUM_VTAPS; x = x + VALUES_PER_MSG) {
          msg.DLC = GENERIC_MSG_LENGTH;
          msg.StdId = ID_MASTER_OCV_MSG;
          msg.Data[0] = i;  //slave id
          msg.Data[1] = x / VALUES_PER_MSG; //row
          msg.Data[2] = extract_MSB(bms.vtaps.ir[i][x]);
          msg.Data[3] = extract_LSB(bms.vtaps.ir[i][x]);
          if (x + 1 < NUM_VTAPS) {
            msg.Data[4] = extract_MSB(bms.vtaps.ir[i][x + 1]);
            msg.Data[5] = extract_LSB(bms.vtaps.ir[i][x + 1]);
          } else {
            msg.Data[4] = 0;
            msg.Data[5] = 0;
          }
          if (x + 2 < NUM_VTAPS) {
            msg.Data[6] = extract_MSB(bms.vtaps.ir[i][x + 2]);
            msg.Data[7] = extract_LSB(bms.vtaps.ir[i][x + 2]);
          } else {
            msg.Data[6] = 0;
            msg.Data[7] = 0;
          }
          
          if (xQueueSendToBack(bms.q_tx_dcan, &msg, 100) != pdPASS) {
            status = FAILURE;
          }
        }
      }
      break;
  }
  
  return status;
}




