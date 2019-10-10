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

- task_Slave_WDawg
- task_txBmsCan
- task_BmsCanProcess
- task_txDcan
- task_DcanProcess
- task_bms_main
- task_heartbeat
- task_broadcast
- task_error_check
- task_sd_card
- task_getIsense
- task_fan_PWM
- task_coulomb_counting
- task_charging

## **Initialization**
Upon startup, BMS Master will first intialize the peripherals used, namely:

- MX_GPIO_Init()
- MX_CAN1_Init()
- MX_CAN3_Init()
- MX_TIM1_Init()
- MX_SDMMC1_SD_Init() (Will be changed to SDIO)

CAN1 is used as DCAN which receives and transmits data from the GUI to BMS Master. CAN3 is used as BMS_CAN which receives data from the BMS SLAVE.

TIM is used to set the duty cycle of the fan using Pulse Width Modulation (PWM) depending on the temperature reading of the slaves.

SDIO is used to read and write from the SD Card on BMS Master.

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
  
- Case 2: `BME_RESET`. Assert `bms.fault.clear`, which will reset all the faults of the BMS by `task_bms_main`.

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
  - `BROAD_CONFIG` (if ir, ocv, temp, volt and macro are enabled)
  - `PASSIVE_EN` (is passive balancing is enabled)

