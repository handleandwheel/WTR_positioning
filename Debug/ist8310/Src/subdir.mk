################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ist8310/Src/ist8310driver.c \
../ist8310/Src/ist8310driver_middleware.c 

OBJS += \
./ist8310/Src/ist8310driver.o \
./ist8310/Src/ist8310driver_middleware.o 

C_DEPS += \
./ist8310/Src/ist8310driver.d \
./ist8310/Src/ist8310driver_middleware.d 


# Each subdirectory must supply rules for building sources it contributes
ist8310/Src/%.o: ../ist8310/Src/%.c ist8310/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/bsp/board" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/bmi088/Inc" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/bmi088/Src" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/ist8310/Inc" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/ist8310/Src" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/pqy13/Inc" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/pqy13/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

