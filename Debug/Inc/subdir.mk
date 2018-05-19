################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/fonts.c \
../Inc/ssd1306.c 

OBJS += \
./Inc/fonts.o \
./Inc/ssd1306.o 

C_DEPS += \
./Inc/fonts.d \
./Inc/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/%.o: ../Inc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/wojti/Documents/PTM/MiniGreenHouse/MiniGreenHouse/Inc" -I"C:/Users/wojti/Documents/PTM/MiniGreenHouse/MiniGreenHouse/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/wojti/Documents/PTM/MiniGreenHouse/MiniGreenHouse/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/wojti/Documents/PTM/MiniGreenHouse/MiniGreenHouse/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/wojti/Documents/PTM/MiniGreenHouse/MiniGreenHouse/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


