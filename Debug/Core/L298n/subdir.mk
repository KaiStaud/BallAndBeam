################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/L298n/L298N.cpp \
../Core/L298n/L298NX2.cpp 

OBJS += \
./Core/L298n/L298N.o \
./Core/L298n/L298NX2.o 

CPP_DEPS += \
./Core/L298n/L298N.d \
./Core/L298n/L298NX2.d 


# Each subdirectory must supply rules for building sources it contributes
Core/L298n/%.o Core/L298n/%.su: ../Core/L298n/%.cpp Core/L298n/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-L298n

clean-Core-2f-L298n:
	-$(RM) ./Core/L298n/L298N.d ./Core/L298n/L298N.o ./Core/L298n/L298N.su ./Core/L298n/L298NX2.d ./Core/L298n/L298NX2.o ./Core/L298n/L298NX2.su

.PHONY: clean-Core-2f-L298n

