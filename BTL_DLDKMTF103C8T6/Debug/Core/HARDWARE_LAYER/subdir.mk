################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/HARDWARE_LAYER/interrupt.c \
../Core/HARDWARE_LAYER/motor.c \
../Core/HARDWARE_LAYER/pid.c \
../Core/HARDWARE_LAYER/read_tick.c \
../Core/HARDWARE_LAYER/reset_data.c 

OBJS += \
./Core/HARDWARE_LAYER/interrupt.o \
./Core/HARDWARE_LAYER/motor.o \
./Core/HARDWARE_LAYER/pid.o \
./Core/HARDWARE_LAYER/read_tick.o \
./Core/HARDWARE_LAYER/reset_data.o 

C_DEPS += \
./Core/HARDWARE_LAYER/interrupt.d \
./Core/HARDWARE_LAYER/motor.d \
./Core/HARDWARE_LAYER/pid.d \
./Core/HARDWARE_LAYER/read_tick.d \
./Core/HARDWARE_LAYER/reset_data.d 


# Each subdirectory must supply rules for building sources it contributes
Core/HARDWARE_LAYER/%.o Core/HARDWARE_LAYER/%.su Core/HARDWARE_LAYER/%.cyclo: ../Core/HARDWARE_LAYER/%.c Core/HARDWARE_LAYER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-HARDWARE_LAYER

clean-Core-2f-HARDWARE_LAYER:
	-$(RM) ./Core/HARDWARE_LAYER/interrupt.cyclo ./Core/HARDWARE_LAYER/interrupt.d ./Core/HARDWARE_LAYER/interrupt.o ./Core/HARDWARE_LAYER/interrupt.su ./Core/HARDWARE_LAYER/motor.cyclo ./Core/HARDWARE_LAYER/motor.d ./Core/HARDWARE_LAYER/motor.o ./Core/HARDWARE_LAYER/motor.su ./Core/HARDWARE_LAYER/pid.cyclo ./Core/HARDWARE_LAYER/pid.d ./Core/HARDWARE_LAYER/pid.o ./Core/HARDWARE_LAYER/pid.su ./Core/HARDWARE_LAYER/read_tick.cyclo ./Core/HARDWARE_LAYER/read_tick.d ./Core/HARDWARE_LAYER/read_tick.o ./Core/HARDWARE_LAYER/read_tick.su ./Core/HARDWARE_LAYER/reset_data.cyclo ./Core/HARDWARE_LAYER/reset_data.d ./Core/HARDWARE_LAYER/reset_data.o ./Core/HARDWARE_LAYER/reset_data.su

.PHONY: clean-Core-2f-HARDWARE_LAYER

