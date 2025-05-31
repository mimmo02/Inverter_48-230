################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/InverterControlFunctions.c \
../Core/Src/display_uart_console.c \
../Core/Src/ll_can.c \
../Core/Src/log_meas.c \
../Core/Src/main.c \
../Core/Src/my_solutions.c \
../Core/Src/power_converter_control_system.c \
../Core/Src/process_1kHz_control.c \
../Core/Src/sample_solutions.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c 

OBJS += \
./Core/Src/InverterControlFunctions.o \
./Core/Src/display_uart_console.o \
./Core/Src/ll_can.o \
./Core/Src/log_meas.o \
./Core/Src/main.o \
./Core/Src/my_solutions.o \
./Core/Src/power_converter_control_system.o \
./Core/Src/process_1kHz_control.o \
./Core/Src/sample_solutions.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o 

C_DEPS += \
./Core/Src/InverterControlFunctions.d \
./Core/Src/display_uart_console.d \
./Core/Src/ll_can.d \
./Core/Src/log_meas.d \
./Core/Src/main.d \
./Core/Src/my_solutions.d \
./Core/Src/power_converter_control_system.d \
./Core/Src/process_1kHz_control.d \
./Core/Src/sample_solutions.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/InverterControlFunctions.cyclo ./Core/Src/InverterControlFunctions.d ./Core/Src/InverterControlFunctions.o ./Core/Src/InverterControlFunctions.su ./Core/Src/display_uart_console.cyclo ./Core/Src/display_uart_console.d ./Core/Src/display_uart_console.o ./Core/Src/display_uart_console.su ./Core/Src/ll_can.cyclo ./Core/Src/ll_can.d ./Core/Src/ll_can.o ./Core/Src/ll_can.su ./Core/Src/log_meas.cyclo ./Core/Src/log_meas.d ./Core/Src/log_meas.o ./Core/Src/log_meas.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/my_solutions.cyclo ./Core/Src/my_solutions.d ./Core/Src/my_solutions.o ./Core/Src/my_solutions.su ./Core/Src/power_converter_control_system.cyclo ./Core/Src/power_converter_control_system.d ./Core/Src/power_converter_control_system.o ./Core/Src/power_converter_control_system.su ./Core/Src/process_1kHz_control.cyclo ./Core/Src/process_1kHz_control.d ./Core/Src/process_1kHz_control.o ./Core/Src/process_1kHz_control.su ./Core/Src/sample_solutions.cyclo ./Core/Src/sample_solutions.d ./Core/Src/sample_solutions.o ./Core/Src/sample_solutions.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su

.PHONY: clean-Core-2f-Src

