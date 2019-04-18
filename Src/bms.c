/*
 * bms.c
 *
 *  Created on: Feb 11, 2019
 *      Author: Matt Flanagan
 */
#include "bms.h"

Success_t temp_probe();
Success_t volt_probe();

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
    fault = NORMAL;
    
    //find the high/low voltage
    volt_probe();
    //find the high/low temp
    temp_probe();
    
    if (bms.state == NORMAL_OP || bms.state == ERROR_BMS) {
      if (bms.fault.charg_en == FAULTED ||
          bms.fault.discharg_en == FAULTED ||
          bms.fault.overtemp == FAULTED ||
          bms.fault.undertemp == FAULTED ||
          bms.fault.overvolt == FAULTED ||
          bms.fault.undervolt == FAULTED ||
					bms.fault.DOC == FAULTED ||
					bms.fault.COC == FAULTED) {
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
        if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
          bms.fault.overall = FAULTED;
          xSemaphoreGive(bms.fault.sem);
        }
        if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
          bms.state = ERROR_BMS;
          xSemaphoreGive(bms.state_sem); //release sem
        }
      } else {
        if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
          bms.fault.overall = NORMAL;
          xSemaphoreGive(bms.fault.sem);
        }
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
*     that freeRTOS will run. These tasks are critical to BMS functionality
*
***************************************************************************/
void initRTOSObjects() {
  uint8_t i = 0;
  
  //define q's
  bms.q_rx_bmscan = xQueueCreate(BMSCAN_TX_Q_SIZE, sizeof(CanTxMsgTypeDef));
  bms.q_tx_bmscan = xQueueCreate(BMSCAN_RX_Q_SIZE, sizeof(CanRxMsgTypeDef));
  bms.q_rx_dcan   = xQueueCreate(DCAN_RX_Q_SIZE, sizeof(CanRxMsgTypeDef));
  bms.q_tx_dcan   = xQueueCreate(DCAN_RX_Q_SIZE, sizeof(CanRxMsgTypeDef));
  
  //start tasks
  xTaskCreate(task_Slave_WDawg, "Master WDawg", WDAWG_STACK_SIZE, NULL, WDAWG_PRIORITY, NULL);
  xTaskCreate(task_txBmsCan, "Transmit BmsCan", BMSCAN_TX_STACK_SIZE, NULL, BMSCAN_TX_PRIORITY, NULL);
  xTaskCreate(task_BmsCanProcess, "BMS Can RX", BMSCAN_RX_STACK_SIZE, NULL, BMSCAN_TX_PRIORITY, NULL);
  xTaskCreate(task_txDcan, "transmit Dcan", DCAN_TX_STACK_SIZE, NULL, DCAN_TX_PRIORITY, NULL);
  xTaskCreate(task_DcanProcess, "Dcan Process", DCAN_RX_STACK_SIZE, NULL, DCAN_RX_PRIORITY, NULL);
  xTaskCreate(task_bms_main, "Main Task", BMS_MAIN_STACK_SIZE, NULL, BMS_MAIN_PRIORITY, NULL);
  xTaskCreate(task_heartbeat, "Heartbeat", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_PRIORITY, NULL);
  xTaskCreate(task_broadcast, "Broadcasting Info on DCAN", BROADCAST_STACK_SIZE, NULL,
              BROADCAST_PRIORITY, bms.normal_op_tasks[i++]);
  xTaskCreate(task_error_check, "Error Check", ERROR_CHECK_STACK_SIZE, NULL,
              ERROR_CHECK_RATE_PRIORITY, bms.normal_op_tasks[i++]);
//  xTaskCreate(task_sd_card, "SD Card", 128, NULL, 1, NULL);
  xTaskCreate(task_getIsense, "ADC Current Sense", ADC_STACK_SIZE, NULL, ADC_PRIORITY, NULL);
  xTaskCreate(task_demo_PWM, "PWM DEMO", ADC_STACK_SIZE, NULL, ADC_PRIORITY, NULL);
  xTaskCreate(task_charging, "Charging", CHARG_STACK_SIZE, NULL, CHARG_PRIORITY, NULL);
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
*       1.flag_t mode Asserted = Reset, Deasserted = Init
*
*      Global Dependents:
*       1. bms
*
*     Function Description: Initialize the BMS structure
***************************************************************************/
void initBMSobject(flag_t mode) {
  uint8_t i = 0;
  uint8_t x = 0;
  
  if (mode == DEASSERTED) {
    bms.state_sem = xSemaphoreCreateBinary();
    bms.params.sem = xSemaphoreCreateBinary();
    bms.fault.sem = xSemaphoreCreateBinary();
    bms.macros.sem = xSemaphoreCreateBinary();
    
    xSemaphoreGive(bms.state_sem);
    xSemaphoreGive(bms.fault.sem);
    xSemaphoreGive(bms.params.sem);
    xSemaphoreGive(bms.macros.sem);
  }
  
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
  
  bms.params.passive_en = DEASSERTED;

  bms.params.ir_msg_rate = BROADCAST_MS * 10;
	bms.params.ocv_msg_rate = BROADCAST_MS * 10;
	bms.params.temp_msg_rate = BROADCAST_MS * 10;
	bms.params.volt_msg_rate = BROADCAST_MS * 10;
	bms.params.macro_msg_rate = BROADCAST_MS;

  bms.fault.charg_en = NORMAL;
  bms.fault.discharg_en = NORMAL;
  bms.fault.overtemp = NORMAL;
  bms.fault.undertemp = NORMAL;
  bms.fault.overvolt = NORMAL;
  bms.fault.undervolt = NORMAL;
  bms.fault.overall = NORMAL;
  bms.fault.COC = NORMAL;
  bms.fault.DOC = NORMAL;
  
  bms.macros.soc = 0;
  bms.macros.pack_i.ch1_low_current = 0;
  bms.macros.pack_i.ch2_high_current = 0;
  bms.macros.pack_volt = 0;
  bms.macros.soh = 0;
  bms.macros.high_temp.val = LIMIT_TEMP_LOW;
  bms.macros.low_temp.val = LIMIT_VOLT_HIGH;
  bms.macros.avg_temp = AVG_TEMP_INIT;
  bms.macros.high_volt.val = LIMIT_VOLT_LOW;
  bms.macros.low_volt.val = LIMIT_VOLT_HIGH;
  
  bms.macros.high_temp.index[0] = 0;
  bms.macros.high_temp.index[1] = 0;
  bms.macros.high_volt.index[0] = 0;
  bms.macros.high_volt.index[1] = 0;
  bms.macros.low_temp.index[0] = 0;
  bms.macros.low_temp.index[1] = 0;
  bms.macros.low_volt.index[0] = 0;
  bms.macros.low_volt.index[1] = 0;
  
  for (i = 0; i < NUM_SLAVES; i++) {
    bms.fault.slave[i].connected = FAULTED;
    bms.fault.slave[i].temp_sens = NORMAL;
    bms.fault.slave[i].volt_sens = NORMAL;
    if (mode == DEASSERTED) {
      bms.vtaps.sem = xSemaphoreCreateBinary();
      bms.temp.sem = xSemaphoreCreateBinary();
      xSemaphoreGive(bms.vtaps.sem);
      xSemaphoreGive(bms.temp.sem);
    }
    
    //initialize all vtap data
    for (x = 0; x < NUM_VTAPS; x ++) {
      bms.vtaps.data[i][x] = VOLT_LOW_IMPOS;
      bms.vtaps.ocv[i][x] = VOLT_LOW_IMPOS;
      bms.vtaps.ir[i][x] = 0;
    }
    for (x = 0; x < NUM_TEMP; x ++) {
      bms.temp.data[i][x] = TEMP_LOW_IMPOS;
    }
  }
  
  for (i = 0; i < NUM_SLAVES; i ++) {
    if (mode == DEASSERTED) {
      wdawg[i].sem = xSemaphoreCreateBinary();
      xSemaphoreGive(wdawg[i].sem); //allows it to be taken
    }
    wdawg[i].last_msg = 0;
  }
  
  if (mode == DEASSERTED) {
    periph.bmscan           = &hcan3;
    //  periph.chargcan         = &hcan2; //todo fix this
    periph.chargcan         = NULL;
    periph.dcan             = &hcan1;
    periph.hdma_sdmmc1_rx   = &hdma_sdmmc1_rx;
    periph.hdma_sdmmc1_tx   = &hdma_sdmmc1_tx;
    periph.hsd1             = &hsd1;
    periph.i_adc            = &hadc1;
    periph.tim              = &htim1;
  }
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
  uint8_t i = 0;
  
  TickType_t time_init = 0;
//  debug_lights(0,0,0,1);
//  HAL_Delay(500);
//  debug_lights(0,0,1,1);
//  HAL_Delay(500);
//  debug_lights(0,1,1,1);
//  HAL_Delay(500);
//  debug_lights(1,1,1,1);
  bms.state = INIT;
  initBMSobject(DEASSERTED);
  while (1) {
    time_init = xTaskGetTickCount();
    i++;
    switch (bms.state) {
      case INIT:
        debug_lights(0, 0, 0, 1);
        initBMSobject(ASSERTED);
        //TODO: do self checks
        //TODO: confirm current sense
        //TODO: confirm SD card connection
        HAL_GPIO_WritePin(SDC_BMS_FAULT_GPIO_Port, SDC_BMS_FAULT_Pin, GPIO_PIN_RESET); //close SDC
        if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
          bms.state = BMS_CONNECT;
          xSemaphoreGive(bms.state_sem); //release sem
        }
        break;
      case BMS_CONNECT:
        debug_lights(0, 0, 1, 0);
        //establish connections with all slaves
        //send wakeup signal and poll until all slaves are connected
        while (slaves_connected() == FAILURE) {
          power_cmd_slaves(POWER_ON);
          vTaskDelay(DELAY_SLAVE_CON);
        }
        if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
          bms.state = NORMAL_OP;
          xSemaphoreGive(bms.state_sem); //release sem
        }
        break;
      case NORMAL_OP:
        debug_lights(0, 0, 1, 1);
        //TODO: read from all of the sensors
        //TODO: manage passive balancing if necessary
        break;
      case ERROR_BMS:
        debug_lights(0, 1, 0, 0);
        //TODO: kill all non critical tasks
        if (bms.fault.overall == NORMAL) {
          //Error has since corrected itself go back to normal operation
          HAL_GPIO_WritePin(SDC_BMS_FAULT_GPIO_Port, SDC_BMS_FAULT_Pin, GPIO_PIN_RESET); //open SDC
          if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
            bms.state = NORMAL_OP;
            xSemaphoreGive(bms.state_sem); //release sem
          }
        } else if (bms.fault.clear == DEASSERTED) {
          HAL_GPIO_WritePin(SDC_BMS_FAULT_GPIO_Port, SDC_BMS_FAULT_Pin, GPIO_PIN_SET); //open SDC
          send_faults();
          vTaskDelay(SEND_ERROR_DELAY);
        } else {
          clear_faults();
          if (xSemaphoreTake(bms.state_sem, TIMEOUT) == pdPASS) {
            bms.state = INIT;
            xSemaphoreGive(bms.state_sem); //release sem
          }
        }
        break;
      case SHUTDOWN:
        debug_lights(0, 1, 0, 1);
        //TODO: kill all non critical tasks
        power_cmd_slaves(POWER_OFF);
        //TODO: finish all sd card writing
        break;
      default:
        //never can get here
        break;
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
Success_t power_cmd_slaves(powercmd_t poweron) {
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
*     Name of Function: slaves_connected
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
Success_t slaves_connected() {
  uint8_t i = 0;
  Success_t success = SUCCESSFUL;
  for (i = 0; i < NUM_SLAVES; i++) {
    if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdTRUE) {
      if (bms.fault.slave[i].connected == FAULTED) {
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
Success_t clear_faults() {
  uint8_t i = 0;
  bms.fault.clear = DEASSERTED; //clear the flag
  bms.fault.charg_en = NORMAL;
  bms.fault.discharg_en = NORMAL;
  bms.fault.overtemp = NORMAL;
  bms.fault.undertemp = NORMAL;
  bms.fault.overvolt = NORMAL;
  bms.fault.undervolt = NORMAL;
  bms.fault.COC = NORMAL;
  bms.fault.DOC = NORMAL;
  
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
Success_t send_faults() {
  CanTxMsgTypeDef msg;
  uint8_t i = 0;
  uint8_t x = 0;
  msg.IDE = CAN_ID_STD;
  msg.RTR = CAN_RTR_DATA;
  msg.DLC = NUM_SLAVES + 1; //one for the macro faults
  msg.StdId = ID_MASTER_ERROR_MSG;
  
  //macro faults
  msg.Data[0] = bitwise_or(FAULT_CHARGE_EN_SHIFT, FAULT_CHARGE_EN_MASK, bms.fault.COC);
  msg.Data[0] |= bitwise_or(FAULT_DISCHARGE_EN_SHIFT, FAULT_DISCHARGE_EN_MASK, bms.fault.DOC);
  msg.Data[0] |= bitwise_or(FAULT_OVERTEMP_SHIFT, FAULT_OVERTEMP_MASK, bms.fault.overtemp);
  msg.Data[0] |= bitwise_or(FAULT_OVERVOLT_SHIFT, FAULT_OVERVOLT_MASK, bms.fault.overvolt);
  msg.Data[0] |= bitwise_or(FAULT_UNDERVOLT_SHIFT, FAULT_UNDERVOLT_MASK, bms.fault.undervolt);
  msg.Data[0] |= bitwise_or(FAULT_UNDERTEMP_SHIFT, FAULT_UNDERTEMP_MASK, bms.fault.undertemp);
  
  //module specific faults
  for (i = 1; i < (NUM_SLAVES + 1); i++) {
    //low module
    msg.Data[i] = bitwise_or(FAULT_MODL_CON_SHIFT, FAULT_MODL_CON_MASK, bms.fault.slave[x].connected);
    msg.Data[i] |= bitwise_or(FAULT_MODL_TEMP_SHIFT, FAULT_MODL_TEMP_MASK,
                              bms.fault.slave[x].temp_sens);
    msg.Data[i] |= bitwise_or(FAULT_MODL_VOLT_SHIFT, FAULT_MODL_VOLT_MASK,
                              bms.fault.slave[x].volt_sens);
                              
    //high module
    if (x + 1 < NUM_SLAVES) {
      msg.Data[i] |= bitwise_or(FAULT_MODH_CON_SHIFT, FAULT_MODH_CON_MASK,
                                bms.fault.slave[x + 1].connected);
      msg.Data[i] |= bitwise_or(FAULT_MODH_TEMP_SHIFT, FAULT_MODH_TEMP_MASK,
                                bms.fault.slave[x + 1].temp_sens);
      msg.Data[i] |= bitwise_or(FAULT_MODH_VOLT_SHIFT, FAULT_MODH_VOLT_MASK,
                                bms.fault.slave[x + 1].volt_sens);
    }
  }
  
  xQueueSendToBack(bms.q_tx_dcan, &msg, 100); //todo DCAN
  return SUCCESSFUL;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: volt_probe
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: Success status
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*       1. bms.volt.data
*
*     Function Description: goes through all the current voltages and updates
*     the location and value of the highest/lowest cells
***************************************************************************/
Success_t volt_probe() {
  Success_t status = FAILURE;
  uint8_t i = 0;
  uint8_t x = 0;
  cell_volt_t temp_max;
  cell_volt_t temp_low;
  
  temp_max.index[0] = 0;
  temp_max.index[1] = 0;
  temp_max.val = VOLT_LOW_IMPOS;
  temp_low.index[0] = 0;
  temp_low.index[1] = 0;
  temp_low.val = VOLT_HIGH_IMPOS;
  
  //find the highest and lowest values
  for (i = 0; i < NUM_SLAVES; i++) {
    for (x = 0; x < NUM_VTAPS; x++) {
      if (bms.vtaps.data[i][x] != VOLT_LOW_IMPOS) {
        //valid voltage data
        if (bms.vtaps.data[i][x] > temp_max.val) {
          temp_max.val = bms.vtaps.data[i][x];
          temp_max.index[0] = i;
          temp_max.index[1] = x;
        }
        
        if (bms.vtaps.data[i][x] < temp_low.val) {
          temp_low.val = bms.vtaps.data[i][x];
          temp_low.index[0] = i;
          temp_low.index[1] = x;
        }
      }
    }
  }
  
  //safety check
  //undervolt check
  if (temp_low.val < bms.params.volt_low_lim) {
    if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
      bms.fault.undervolt = FAULTED;
      xSemaphoreGive(bms.fault.sem);
    } else {
      status = FAILURE;
    }
  } else {
    if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
      bms.fault.undervolt = NORMAL;
      xSemaphoreGive(bms.fault.sem);
    } else {
      status = FAILURE;
    }
  }
  
  if (temp_max.val > bms.params.volt_high_lim) {
    if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
      bms.fault.overvolt = FAULTED;
      xSemaphoreGive(bms.fault.sem);
    } else {
      status = FAILURE;
    }
  } else {
    if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
      bms.fault.overvolt = NORMAL;
      xSemaphoreGive(bms.fault.sem);
    } else {
      status = FAILURE;
    }
  }
  
  //update
  if (xSemaphoreTake(bms.macros.sem, TIMEOUT) == pdPASS) {
    status = SUCCESS;
    bms.macros.high_volt.index[0] = temp_max.index[0];
    bms.macros.high_volt.index[1] = temp_max.index[1];
    bms.macros.high_volt.val = temp_max.val;
    
    bms.macros.low_volt.index[0] = temp_low.index[0];
    bms.macros.low_volt.index[1] = temp_low.index[1];
    bms.macros.low_volt.val = temp_low.val;
    
    xSemaphoreGive(bms.macros.sem);
  } else {
    status = FAILURE;
  }
  
  return status;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: temp_probe
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: Success status
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*       1. bms.temp.data
*
*     Function Description: updates the highest/lowest temp cell and index from
*     the array
***************************************************************************/
Success_t temp_probe() {
  Success_t status = FAILURE;
  uint8_t i = 0;
  uint8_t x = 0;
  cell_temp_t temp_max;
  cell_temp_t temp_low;
  int16_t temp_avg = 0;
  
  temp_max.index[0] = 0;
  temp_max.index[1] = 0;
  temp_max.val = TEMP_LOW_IMPOS;
  temp_low.index[0] = 0;
  temp_low.index[1] = 0;
  temp_low.val = TEMP_HIGH_IMPOS;
  
  //find the highest and lowest values
  for (i = 0; i < NUM_SLAVES; i++) {
    for (x = 0; x < NUM_TEMP; x++) {
      if (bms.temp.data[i][x] != TEMP_LOW_IMPOS) {
        //calculate average temperature across the module
        temp_avg += bms.temp.data[i][x];

        //valid temperature data
        if (bms.temp.data[i][x] > temp_max.val) {
          temp_max.val = bms.temp.data[i][x];
          temp_max.index[0] = i;
          temp_max.index[1] = x;
        }
        
        if (bms.temp.data[i][x] < temp_low.val) {
          temp_low.val = bms.temp.data[i][x];
          temp_low.index[0] = i;
          temp_low.index[1] = x;
        }
      }
    }
  }
  //calculate average temperature across the module
  temp_avg = temp_avg / NUM_SLAVES / NUM_TEMP;
  bms.macros.avg_temp = temp_avg;
  
  //safety check
  //undervolt check
  if ((temp_low.val < bms.params.temp_low_lim) && (temp_low.val != (int16_t) TEMP_LOW_IMPOS)) {
    if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
      bms.fault.undertemp = FAULTED;
      xSemaphoreGive(bms.fault.sem);
    } else {
      status = FAILURE;
    }
  } else {
    if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
      bms.fault.undertemp = NORMAL;
      xSemaphoreGive(bms.fault.sem);
    } else {
      status = FAILURE;
    }
  }
  
  if (temp_max.val > bms.params.temp_high_lim) {
    if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
      bms.fault.overtemp = FAULTED;
      xSemaphoreGive(bms.fault.sem);
    } else {
      status = FAILURE;
    }
  } else {
    if (xSemaphoreTake(bms.fault.sem, TIMEOUT) == pdPASS) {
      bms.fault.overtemp = NORMAL;
      xSemaphoreGive(bms.fault.sem);
    } else {
      status = FAILURE;
    }
  }
  
  //update
  if (xSemaphoreTake(bms.macros.sem, TIMEOUT) == pdPASS) {
    status = SUCCESS;
    bms.macros.high_temp.index[0] = temp_max.index[0];
    bms.macros.high_temp.index[1] = temp_max.index[1];
    bms.macros.high_temp.val = temp_max.val;
    
    bms.macros.low_temp.index[0] = temp_low.index[0];
    bms.macros.low_temp.index[1] = temp_low.index[1];
    bms.macros.low_temp.val = temp_low.val;
    
    xSemaphoreGive(bms.macros.sem);
  } else {
    status = FAILURE;
  }
  
  return status;
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

void debug_lights(flag_t orange, flag_t red, flag_t green, flag_t blue) {
  if (orange == ASSERTED) {
    HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
  }
  if (red == ASSERTED) {
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  }
  if (green == ASSERTED) {
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
  }
  if (blue == ASSERTED) {
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
  }
}






