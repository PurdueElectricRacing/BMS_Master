################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc_isense.c \
../Src/bms.c \
../Src/bms_can.c \
../Src/bsp_driver_sd.c \
../Src/charging.c \
../Src/coulomb_counting.c \
../Src/dcan.c \
../Src/fatfs.c \
../Src/fatfs_platform.c \
../Src/freertos.c \
../Src/main.c \
../Src/sd_diskio.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_tim.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c \
../Src/tim_fan_pwm.c 

OBJS += \
./Src/adc_isense.o \
./Src/bms.o \
./Src/bms_can.o \
./Src/bsp_driver_sd.o \
./Src/charging.o \
./Src/coulomb_counting.o \
./Src/dcan.o \
./Src/fatfs.o \
./Src/fatfs_platform.o \
./Src/freertos.o \
./Src/main.o \
./Src/sd_diskio.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_tim.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o \
./Src/tim_fan_pwm.o 

C_DEPS += \
./Src/adc_isense.d \
./Src/bms.d \
./Src/bms_can.d \
./Src/bsp_driver_sd.d \
./Src/charging.d \
./Src/coulomb_counting.d \
./Src/dcan.d \
./Src/fatfs.d \
./Src/fatfs_platform.d \
./Src/freertos.d \
./Src/main.d \
./Src/sd_diskio.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_tim.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d \
./Src/tim_fan_pwm.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"E:/PER/BMS_Master/BMS_Master_Test/Inc" -I"E:/PER/BMS_Master/BMS_Master_Test/Drivers/STM32F4xx_HAL_Driver/Inc" -I"E:/PER/BMS_Master/BMS_Master_Test/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"E:/PER/BMS_Master/BMS_Master_Test/Middlewares/Third_Party/FreeRTOS/Source/include" -I"E:/PER/BMS_Master/BMS_Master_Test/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"E:/PER/BMS_Master/BMS_Master_Test/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"E:/PER/BMS_Master/BMS_Master_Test/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"E:/PER/BMS_Master/BMS_Master_Test/Drivers/CMSIS/Include" -I"E:/PER/BMS_Master/BMS_Master_Test/Middlewares/Third_Party/FatFs/src"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


