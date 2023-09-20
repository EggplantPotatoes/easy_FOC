################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user_app/ADC_encoder.c \
../user_app/BEMF.c \
../user_app/IIC_encoder.c \
../user_app/PWM_Encoder.c \
../user_app/current_sample.c \
../user_app/data_filter.c \
../user_app/debug.c \
../user_app/foc.c \
../user_app/motor_bsp.c \
../user_app/motor_function.c \
../user_app/pid_control.c \
../user_app/test.c 

OBJS += \
./user_app/ADC_encoder.o \
./user_app/BEMF.o \
./user_app/IIC_encoder.o \
./user_app/PWM_Encoder.o \
./user_app/current_sample.o \
./user_app/data_filter.o \
./user_app/debug.o \
./user_app/foc.o \
./user_app/motor_bsp.o \
./user_app/motor_function.o \
./user_app/pid_control.o \
./user_app/test.o 

C_DEPS += \
./user_app/ADC_encoder.d \
./user_app/BEMF.d \
./user_app/IIC_encoder.d \
./user_app/PWM_Encoder.d \
./user_app/current_sample.d \
./user_app/data_filter.d \
./user_app/debug.d \
./user_app/foc.d \
./user_app/motor_bsp.d \
./user_app/motor_function.d \
./user_app/pid_control.d \
./user_app/test.d 


# Each subdirectory must supply rules for building sources it contributes
user_app/%.o user_app/%.su user_app/%.cyclo: ../user_app/%.c user_app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"F:/Brushless_Motor/Software/ST_motor/My_ST_Motor/STM32_IDE_code/easy_FOC_STM32IDE/user_app" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-user_app

clean-user_app:
	-$(RM) ./user_app/ADC_encoder.cyclo ./user_app/ADC_encoder.d ./user_app/ADC_encoder.o ./user_app/ADC_encoder.su ./user_app/BEMF.cyclo ./user_app/BEMF.d ./user_app/BEMF.o ./user_app/BEMF.su ./user_app/IIC_encoder.cyclo ./user_app/IIC_encoder.d ./user_app/IIC_encoder.o ./user_app/IIC_encoder.su ./user_app/PWM_Encoder.cyclo ./user_app/PWM_Encoder.d ./user_app/PWM_Encoder.o ./user_app/PWM_Encoder.su ./user_app/current_sample.cyclo ./user_app/current_sample.d ./user_app/current_sample.o ./user_app/current_sample.su ./user_app/data_filter.cyclo ./user_app/data_filter.d ./user_app/data_filter.o ./user_app/data_filter.su ./user_app/debug.cyclo ./user_app/debug.d ./user_app/debug.o ./user_app/debug.su ./user_app/foc.cyclo ./user_app/foc.d ./user_app/foc.o ./user_app/foc.su ./user_app/motor_bsp.cyclo ./user_app/motor_bsp.d ./user_app/motor_bsp.o ./user_app/motor_bsp.su ./user_app/motor_function.cyclo ./user_app/motor_function.d ./user_app/motor_function.o ./user_app/motor_function.su ./user_app/pid_control.cyclo ./user_app/pid_control.d ./user_app/pid_control.o ./user_app/pid_control.su ./user_app/test.cyclo ./user_app/test.d ./user_app/test.o ./user_app/test.su

.PHONY: clean-user_app

