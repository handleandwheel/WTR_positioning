/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Fusion.h"
#include "ist8310driver.h"
#include "BMI088driver.h"
#include "PQY13driver.h"
#include "CompileConfig.h"
#include "struct_typedef.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
FusionBias fusionBias;
FusionAhrs fusionAhrs;
fp32 samplePeriod = 0.01f;
/* sensitivity of the gyroscope, values are in degrees per second per lsb
  * the range of the gyroscope is 2000, the value in lsb per degrees per second is 16.384
  */
static FusionVector3 gyroscopeSensitivity = {
	.axis.x = 6.10352e-2f,
	.axis.y = 6.10352e-2f,
	.axis.z = 6.10352e-2f,
};
/* sensitivity of the accelerometer, values are in g per lsb
  * the range of the accelerometer is 3G, the value in lsb per g is 10920
  */
static FusionVector3 accelerometerSensitivity = {
	.axis.x = 9.15751e-5f,
	.axis.y = 9.15751e-5f,
	.axis.z = 9.15751e-5f,
};
/* replace these values with actual hard-iron bias in uT if known */
static FusionVector3 hardIronBias = {
	.axis.x = 0.0f,
	.axis.y = 0.0f,
	.axis.z = 0.0f,
};
fp32 gyro[3], accel[3], mag[3], tem;
FusionEulerAngles eulerAngles;
/* USER CODE END Variables */
osThreadId orientationTaskHandle;
osThreadId positionTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartOrientationTask(void const * argument);
void StartPositionTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	while(BMI088_init());
	while(ist8310_init());
	FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod);  // stationary threshold = 0.5 degrees per second
	FusionAhrsInitialise(&fusionAhrs, 0.5f);  // gain = 0.5
	FusionAhrsSetMagneticField(&fusionAhrs, 20.0f, 70.0f);  // valid magnetic field range = 20 uT to 70 uT
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of orientationTask */
  osThreadDef(orientationTask, StartOrientationTask, osPriorityNormal, 0, 1024);
  orientationTaskHandle = osThreadCreate(osThread(orientationTask), NULL);

  /* definition and creation of positionTask */
  osThreadDef(positionTask, StartPositionTask, osPriorityNormal, 0, 400);
  positionTaskHandle = osThreadCreate(osThread(positionTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartOrientationTask */
/**
  * @brief  Function implementing the orientationTask thread.
  * @param  argument: Not used
  * @retval None
  *
  */
/* USER CODE END Header_StartOrientationTask */
void StartOrientationTask(void const * argument)
{
  /* USER CODE BEGIN StartOrientationTask */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000 * samplePeriod;
  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  BMI088_read(gyro, accel, &tem);
	  ist8310_read_mag(mag);
	  // Calibrate gyroscope
	  FusionVector3 uncalibratedGyroscope = {
		  .axis.x = gyro[0], /* replace this value with actual gyroscope x axis measurement in lsb */
		  .axis.y = gyro[1], /* replace this value with actual gyroscope y axis measurement in lsb */
		  .axis.z = gyro[2], /* replace this value with actual gyroscope z axis measurement in lsb */
	  };
	  FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

	  // Calibrate accelerometer
	  FusionVector3 uncalibratedAccelerometer = {
		  .axis.x = accel[0], /* replace this value with actual accelerometer x axis measurement in lsb */
		  .axis.y = accel[1], /* replace this value with actual accelerometer y axis measurement in lsb */
		  .axis.z = accel[2], /* replace this value with actual accelerometer z axis measurement in lsb */
	  };
	  FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	  // Calibrate magnetometer
	  FusionVector3 uncalibratedMagnetometer = {
		  .axis.x = mag[0], /* replace this value with actual magnetometer x axis measurement in uT */
		  .axis.y = mag[1], /* replace this value with actual magnetometer y axis measurement in uT */
		  .axis.z = mag[2], /* replace this value with actual magnetometer z axis measurement in uT */
	  };
	  FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

	  // Update gyroscope bias correction algorithm
	  calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

	  // Update AHRS algorithm
	  FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, samplePeriod);
	  eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);  // task stack size should set to 1024, other wise the stack will overflow and cause bug at this line.
  }
  /* USER CODE END StartOrientationTask */
}

/* USER CODE BEGIN Header_StartPositionTask */
/**
* @brief Function implementing the positionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPositionTask */
void StartPositionTask(void const * argument)
{
  /* USER CODE BEGIN StartPositionTask */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(1);
  }
  /* USER CODE END StartPositionTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
