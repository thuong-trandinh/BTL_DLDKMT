################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/function_layer/control_motor.c \
../Core/function_layer/process_read_tick.c \
../Core/function_layer/setting_PID.c 

OBJS += \
./Core/function_layer/control_motor.o \
./Core/function_layer/process_read_tick.o \
./Core/function_layer/setting_PID.o 

C_DEPS += \
./Core/function_layer/control_motor.d \
./Core/function_layer/process_read_tick.d \
./Core/function_layer/setting_PID.d 


# Each subdirectory must supply rules for building sources it contributes
Core/function_layer/%.o Core/function_layer/%.su Core/function_layer/%.cyclo: ../Core/function_layer/%.c Core/function_layer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-function_layer

clean-Core-2f-function_layer:
	-$(RM) ./Core/function_layer/control_motor.cyclo ./Core/function_layer/control_motor.d ./Core/function_layer/control_motor.o ./Core/function_layer/control_motor.su ./Core/function_layer/process_read_tick.cyclo ./Core/function_layer/process_read_tick.d ./Core/function_layer/process_read_tick.o ./Core/function_layer/process_read_tick.su ./Core/function_layer/setting_PID.cyclo ./Core/function_layer/setting_PID.d ./Core/function_layer/setting_PID.o ./Core/function_layer/setting_PID.su

.PHONY: clean-Core-2f-function_layer

