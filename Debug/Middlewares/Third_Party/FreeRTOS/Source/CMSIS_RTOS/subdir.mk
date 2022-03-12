################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/%.c Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/bmi088/Inc" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/bmi088/Src" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/ist8310/Inc" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/ist8310/Src" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/pqy13/Inc" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/pqy13/Src" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/SensorDriver/board" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/CompileConfig" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/Algorithm/orientation/Src" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/Algorithm/orientation/Inc" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/Algorithm/pose/Inc" -I"C:/Users/liuyu/Desktop/imu_test_win/imu_test/Algorithm/pose/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

