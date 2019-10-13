# **BMS Master** 

## **Overview**
The main functionality of the BMS Master includes:

* Process cell temperature and voltage reading from Slave to determine faults.
* Determine the State of Charge (SoC) and State of Health (SoH) of the cells.
* Transmit data to Graphical User Interface (GUI).
* Receive and process commands from GUI.

Further details of the tasks required to achieve these functionalities will be detailed below.

## **Roadmap**
- [ ] Port the current model from F7 to F4.
- [ ] Incorporate and test SD Card functionality.
- [ ] Add SoC Model comparison and test accuracy of SoC Calculations.
- [ ] Clean up and remove unused functions. (USART, DMA, CAN3).
- [ ] Change all passive balancing to active balancing.

## **Basic Structure**
The BMS slave STM32 uses FreeRTOS to manage scheduling and execution of code. By doing this, the application can run multiple threads "simultaneously" allowing for easier development. 

Upon startup, BMS Master will first go through an initialization phase detailed below. After that, the following tasks will be started:

- `task_Slave_WDawg`
- `task_txBmsCan`
- `task_BmsCanProcess`
- `task_txDcan`
- `task_DcanProcess`
- `task_bms_main`
- `task_heartbeat`
- `task_broadcast`
- `task_error_check`
- `task_sd_card`
- `task_getIsense`
- `task_fan_PWM`
- `task_coulomb_counting`
- `task_charging`

## **Initialization**
Upon startup, BMS Master will first intialize the peripherals used, namely:

- `MX_GPIO_Init()`
- `MX_CAN1_Init()`
- `MX_CAN3_Init()`
- `MX_TIM1_Init()`
- `MX_SDMMC1_SD_Init()` (Will be changed to SDIO)
- `MX_DMA_Init()`
- `MX_ADC1_Init()`

CAN1 is used as DCAN which receives and transmits data from the GUI to BMS Master. CAN3 is used as BMS_CAN which receives data from the BMS SLAVE.

TIM is used to set the duty cycle of the fan using Pulse Width Modulation (PWM) depending on the temperature reading of the slaves.

SDIO is used to read and write from the SD Card on BMS Master.

ADC is used to read battery charging/discharging current value from the current transducer.

## **Tasks**
### **task_Slave_WDawg**
This task monitors the DCAN traffic from each slave module one by one to see if they are still in communication. The following two conditions will set a specific `BMS.slave[i].connected` status = `FAULTED`.

- Slave module's last message is longer than `(6000 / portTICK_RATE_MS) / NUM_SLAVES`.
- Slave module's last message is empty.

### **task_txBmsCan**
This task checks the `tx_bmscan` queue every `CAN_TX_RATE` and sends them out to the GUI.

### **task_BmsCanProcess**
This task reads the `rx_bmscan` queue and process the message accordingly. There are 4 possible types of messages.

- Case 1: `ID_SLAVE_ACK`. Nothing needs to be done here as the `task_Slave_WDawg` is monitoring the connection status of the slaves.
- Case 2: `ID_SLAVE_FAULT`. Extracts the voltage and temperature reading from the CAN message to store in `BMS.slave[i].volt_sens` and `BMS.slave[i].temp_sens`.
- Case 3: `ID_SLAVE_VOLT`. Stores all 21 voltage readings from one slave into a 2D aray `bms.temp.data[slave_number][reading_number]`.
- Case 4: `ID_SLAVE_TEMP`. Stores all 17 temperature readings from one slave into a 2D array `bms.vtaps.data[slave_number][reading_number]`.

### **task_txDcan**
This task checks the `tx_dcan` queue every `CAN_TX_RATE` and sends them out to the GUI.

### **task_DcanProcess**
This task reads the `rx_dcan` queue and process the message accordingly. There are 4 possible types of messages.

- Case 1: `GUI_CMD`. There are 3 possible commands from the GUI.
  - `LOG_DATA`: send data to the SD card.
  - `DELETE`: delete data from the SD card.
  - `CONFIGURE`: enable CAN messages about internal resistance (ir), open circuit voltage (ocv), temperature, voltage or macro.
  
- Case 2: `BMS_RESET`. Assert `bms.fault.clear`, which will reset all the faults of the BMS by `task_bms_main`.

- Case 3: `PARAM_SET`. Set one of the following parameters based on the input from GUI.
  - `TEMP_HIGH_LIMIT`, `TEMP_LOW_LIMIT`
  - `VOLT_HIGH_LIMIT`, `VOLT_LOW_LIMIT`
  - `DISCHARGE_LIMIT`, `CHARGE_LIMIT`
  - `VOLT_MSG_RATE`, `TEMP_MSG_RATE`, `OCV_MSG_RATE`, `IR_MSG_RATE`, `MACRO_MSG_RATE`
  - `PASSIVE_EN` (enable passive balancing)
  
- Case 4: `PARAM_REQ`. Request to read a certain parameter. Here are the possible requests.
  - `TEMP_HIGH_LIMIT`, `TEMP_LOW_LIMIT`
  - `VOLT_HIGH_LIMIT`, `VOLT_LOW_LIMIT`
  - `DISCHARGE_LIMIT`, `CHARGE_LIMIT`
  - `VOLT_MSG_RATE`, `TEMP_MSG_RATE`, `OCV_MSG_RATE`, `IR_MSG_RATE`, `MACRO_MSG_RATE`
  - `SOC_VALUE`, `SOH_VALUE`
  - `PACK_VOLTAGE`, `PACK_CURRENT`
  - `HIGH_TEMP` (highest temperature recorded)
  - `BROAD_CONFIG` (if ir, ocv, temp, volt and macro messages are enabled)
  - `PASSIVE_EN` (if passive balancing is enabled)

### **task_bms_main**
This is the main state transition process of the BMS which intializes all parameters and handle error and shutdown process. At the start, the BMS will initialize all parameters, then go into either one of these states, with `INIT` being the first. This task also add the `bms.state` to the `bms.q_tx_bmscan` from time to time.

- State 1: `INIT`
  - Initializes all the parameters of the BMS.
  - Close the Shut Down Circuit (SDC).
  - Transition into the `BMS_CONNECT` state.
  
- State 2: `BMS_CONNECT`
  - Sends CAN messages to each slave to power on while connection is not established.
  - Transition into the `NORMAL_OP` state.
  
- State 3: `NORMAL_OP`
  - So far nothing yet...
 
- State 4: `ERROR_BMS`
  - Checks `bms.fault.overall` to see if fault is cleared, and go back to `NORMAL_OP` if it is.
  - Checks the `bms.fault.clear` to see if the GUI commands that the faults be cleared. If it is asserted, clear all faults and go back to `INIT`.
  - If clear not asserted and fault is not cleared, open the SDC and send all faults to GUI.
 
- State 5: `SHUTDOWN`
  - Send CAN message to each slave to shutdown.

### **task_heartbeat**
Toggles a gpio at `HEARTBEAT_RATE` to satisfy the watchdog timer.

### **task_broadcast**
Sends CAN messages for available ir, ocv, volt, temp or macro when:
- That specific message type is enabled.
- The counter matches the specific message rate.

### **task_error_check**
This task first probe for the high and low temp and volt then checks the possible places for error and fault the BMS if it occurs. The possible places are:

- `bms_fault`
  - charge_en, discharge_en
  - overtemp, undertemp
  - overvolt, undervolt
  - DOC, SOC
- `bms.fault.slave`
  - connected
  - temp_sense
  - volt_sense

If fault is found, this will send the BMS to the `ERROR_BMS` state and set `bms.fault.overall` to be `FAULTED`.

### **task_getIsense**
This task will do 3 things:

- Reads in 2 current values from the current transducer - channel 1 which goes up to +- 75A and channel 2 which goes to +- 500A.
- Update the `bms.macros.pack_i.ch1_low_current` to current at channel 1 and `bms.macros.pack_i.ch2_low_current` to current at channel 2 value.
- Check for charging or discharging over current fault, and fault the BMS is detected.

### **task_fan_PWM**
This task sets the duty cycle of the pwm which drives the fan according to `bms.macros.temp_avg`.

### **task_coulomb_counting**
This task calculates and updates the SoC and SoH value using the coulomb counting method which takes into account the following factors:

- Instantaneous current
- Open Circuit Voltage
- Self discharing
- Temperature

Further reading about coulomb_counting can be found below.

### **task_charging**
This task scans throught the slaves and if `modules[x] - min_volt > DELTA_VOLT`, send balancing message to the specific slave.

## **References**
[Google Drive Link](https://drive.google.com/drive/u/0/folders/10BhgTpdeEc__XZ9EmfNQqM7rGZ7nbBgL)

[CAN Messages](https://docs.google.com/spreadsheets/d/1AIsM6NNCoJiR2NbtGhZPkIug68hO--Kt/edit#gid=930516311)

[Isense Datasheet](https://www.electronicsdatasheets.com/manufacturers/lem/parts/dhab-s24#datasheet)

[Papers about coulomb counting](https://drive.google.com/drive/u/0/folders/1brrGmAbWCBpCmF5a4roTPAF_cTrg1yqj)

<sub>last updated: 10/13/19 by Tan Li Yon</sub>
