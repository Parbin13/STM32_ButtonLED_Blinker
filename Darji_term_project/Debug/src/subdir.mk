################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/exti-main.c \
../src/main.c \
../src/systick_interrupt_main.c \
../src/systick_polling_main.c 

OBJS += \
./src/exti-main.o \
./src/main.o \
./src/systick_interrupt_main.o \
./src/systick_polling_main.o 

C_DEPS += \
./src/exti-main.d \
./src/main.d \
./src/systick_interrupt_main.d \
./src/systick_polling_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mthumb -mfloat-abi=soft -I"C:/Users/prabi/workspace/Darji_term_project/src" -O0 -g -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


