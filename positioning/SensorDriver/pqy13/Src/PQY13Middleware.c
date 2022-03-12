/*
 * PQY13Middleware.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Liu, Yuming
 */


#include "stm32f4xx_hal.h"
#include "PQY13port.h"

extern SPI_HandleTypeDef hspi2;

void PQY13_ENC1_CSN_ENABLE(void)
{
	HAL_GPIO_WritePin(CSN_ENC1_GPIO_Port, CSN_ENC1_Pin, GPIO_PIN_RESET);
}

void PQY13_ENC1_CSN_DISABLE(void)
{
	HAL_GPIO_WritePin(CSN_ENC1_GPIO_Port, CSN_ENC1_Pin, GPIO_PIN_SET);
}

void PQY13_ENC2_CSN_ENABLE(void)
{
	HAL_GPIO_WritePin(CSN_ENC2_GPIO_Port, CSN_ENC2_Pin, GPIO_PIN_RESET);
}

void PQY13_ENC2_CSN_DISABLE(void)
{
	HAL_GPIO_WritePin(CSN_ENC2_GPIO_Port, CSN_ENC2_Pin, GPIO_PIN_SET);
}

void PQY13_read_single_data(uint8_t txdata, uint8_t *rxdata)
{
	HAL_SPI_Transmit(&hspi2, &txdata, 1, 1000);
	HAL_SPI_Receive(&hspi2, rxdata, 4, 1000);
}
