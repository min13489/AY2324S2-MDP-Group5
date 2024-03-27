################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral_Drivers/Src/oled.c 

OBJS += \
./Peripheral_Drivers/Src/oled.o 

C_DEPS += \
./Peripheral_Drivers/Src/oled.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral_Drivers/Src/%.o Peripheral_Drivers/Src/%.su: ../Peripheral_Drivers/Src/%.c Peripheral_Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"D:/SC2079_MDP/PeripheralDriver/PeripheralDriver/Inc" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Peripheral_Drivers-2f-Src

clean-Peripheral_Drivers-2f-Src:
	-$(RM) ./Peripheral_Drivers/Src/oled.d ./Peripheral_Drivers/Src/oled.o ./Peripheral_Drivers/Src/oled.su

.PHONY: clean-Peripheral_Drivers-2f-Src

