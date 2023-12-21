################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TER_DATABASES/hvbms.c \
../TER_DATABASES/inverter.c \
../TER_DATABASES/te_r23.c \
../TER_DATABASES/ter.c 

OBJS += \
./TER_DATABASES/hvbms.o \
./TER_DATABASES/inverter.o \
./TER_DATABASES/te_r23.o \
./TER_DATABASES/ter.o 

C_DEPS += \
./TER_DATABASES/hvbms.d \
./TER_DATABASES/inverter.d \
./TER_DATABASES/te_r23.d \
./TER_DATABASES/ter.d 


# Each subdirectory must supply rules for building sources it contributes
TER_DATABASES/%.o TER_DATABASES/%.su TER_DATABASES/%.cyclo: ../TER_DATABASES/%.c TER_DATABASES/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/eracing/Documentos/GitHub/TeR_ECU/SOFTWARE/TeR_ECU/TER_DATABASES" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TER_DATABASES

clean-TER_DATABASES:
	-$(RM) ./TER_DATABASES/hvbms.cyclo ./TER_DATABASES/hvbms.d ./TER_DATABASES/hvbms.o ./TER_DATABASES/hvbms.su ./TER_DATABASES/inverter.cyclo ./TER_DATABASES/inverter.d ./TER_DATABASES/inverter.o ./TER_DATABASES/inverter.su ./TER_DATABASES/te_r23.cyclo ./TER_DATABASES/te_r23.d ./TER_DATABASES/te_r23.o ./TER_DATABASES/te_r23.su ./TER_DATABASES/ter.cyclo ./TER_DATABASES/ter.d ./TER_DATABASES/ter.o ./TER_DATABASES/ter.su

.PHONY: clean-TER_DATABASES

