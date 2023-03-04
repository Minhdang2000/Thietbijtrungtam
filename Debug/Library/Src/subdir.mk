################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/Src/Lora.c \
../Library/Src/ST7735_SPI.c \
../Library/Src/fonts.c \
../Library/Src/function.c 

OBJS += \
./Library/Src/Lora.o \
./Library/Src/ST7735_SPI.o \
./Library/Src/fonts.o \
./Library/Src/function.o 

C_DEPS += \
./Library/Src/Lora.d \
./Library/Src/ST7735_SPI.d \
./Library/Src/fonts.d \
./Library/Src/function.d 


# Each subdirectory must supply rules for building sources it contributes
Library/Src/%.o Library/Src/%.su: ../Library/Src/%.c Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/CudeIDE/Doantotnghiep/Trungtam_lora/Library/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Library-2f-Src

clean-Library-2f-Src:
	-$(RM) ./Library/Src/Lora.d ./Library/Src/Lora.o ./Library/Src/Lora.su ./Library/Src/ST7735_SPI.d ./Library/Src/ST7735_SPI.o ./Library/Src/ST7735_SPI.su ./Library/Src/fonts.d ./Library/Src/fonts.o ./Library/Src/fonts.su ./Library/Src/function.d ./Library/Src/function.o ./Library/Src/function.su

.PHONY: clean-Library-2f-Src

