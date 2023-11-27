################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/NHK2024_Library/example/filter/moving_average_filter.c 

OBJS += \
./Core/NHK2024_Library/example/filter/moving_average_filter.o 

C_DEPS += \
./Core/NHK2024_Library/example/filter/moving_average_filter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/NHK2024_Library/example/filter/%.o Core/NHK2024_Library/example/filter/%.su Core/NHK2024_Library/example/filter/%.cyclo: ../Core/NHK2024_Library/example/filter/%.c Core/NHK2024_Library/example/filter/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-NHK2024_Library-2f-example-2f-filter

clean-Core-2f-NHK2024_Library-2f-example-2f-filter:
	-$(RM) ./Core/NHK2024_Library/example/filter/moving_average_filter.cyclo ./Core/NHK2024_Library/example/filter/moving_average_filter.d ./Core/NHK2024_Library/example/filter/moving_average_filter.o ./Core/NHK2024_Library/example/filter/moving_average_filter.su

.PHONY: clean-Core-2f-NHK2024_Library-2f-example-2f-filter

