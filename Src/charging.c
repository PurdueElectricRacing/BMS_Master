/*
 * charging.c
 *
 *  Created on: Apr 15, 2019
 *      Author: matth
 */

#include "charging.h"

void calc_avg_voltage(int* modules, int* min_mod);
void write_passive_msg(uint8_t slave_id, flag_t passive_bal);

void task_charging() {
  TickType_t time_init = 0;
  uint16_t modules[NUM_SLAVES];
  uint16_t min_volt; //0 = slave id, 1 = voltage
  uint8_t x;
  int16_t delta;
  while (1) {
    time_init = xTaskGetTickCount();

    if (bms.params.passive_en == ASSERTED) {
      //passive balancing enabled
      //calculate average voltage of each modules
      calc_avg_voltage(&modules, &min_volt);
      //check if delta greater than DELTA Volts
      for (x = 0; x < NUM_SLAVES; x++) {
        //not the min module check the delta
        delta = modules[x] - min_volt[1];
        if (delta > DELTA_VOLT) {
          //enable passive balancing
          write_passive_msg(x, ASSERTED);
        } else {
          //disable passive balancing
          write_passive_msg(x, DEASSERTED);
        }
      }
    } else {
      //disable passive balancing on all slaves
      for (x = 0; x < NUM_SLAVES; x++) {
        write_passive_msg(x, DEASSERTED);
      }
    }
    vTaskDelayUntil(&time_init, CHARG_RATE);
  }
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: calc_avg_voltage
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. An array of size NUM_SLAVES. Used to set the average voltage of each module
*
*      Global Dependents:
*       1. bms.vtaps
*
*     Function Description: calculates the average voltage of each module and stores
*     that in an array given by the calling function
***************************************************************************/
void calc_avg_voltage(int* modules, int* min_mod) {
  uint8_t x = 0;
  uint8_t i = 0;
  uint16_t volt_avg = 0;
  uint8_t num_volts = 0;
  uint16_t temp_min = VOLT_HIGH_IMPOS;

  for (x = 0; x < NUM_SLAVES; x++) {
    volt_avg = 0;
    num_volts = 0;
    for (i = 0; i < NUM_VTAPS; i++) {
      if (bms.vtaps.data[x][i] != VOLT_LOW_IMPOS) {
        //valid voltage data
        num_volts++;
        volt_avg += bms.vtaps.data[x][i];
      }
    }
    modules[x] = volt_avg / num_volts;
    if (modules[x] < temp_min) {
      //new min voltage found update the temp
      temp_min = modules[x];
    }
  }
  *min_mod = temp_min;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: write_passive_msg
*
*     Programmer's Name: Matt Flanagan
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. slave_id - the id of the receiving slave
*       2. passive_bal - whether to enable or disable passive balancing
*
*      Global Dependents:
*       1. bms.vtaps
*
*     Function Description: sends a can message to enable/disable the passive balancing
***************************************************************************/
void write_passive_msg(uint8_t slave_id, flag_t passive_bal) {
  CanTxMsgTypeDef msg;
  msg.IDE = CAN_ID_STD;
  msg.RTR = CAN_RTR_DATA;
  msg.DLC = 2;
  msg.StdId = ID_MAS_PASSIVE;
  msg.Data[0] = slave_id;
  msg.Data[1] = (uint8_t) passive_bal;

  xQueueSendToBack(bms.q_tx_bmscan, &msg, TIMEOUT);
}
