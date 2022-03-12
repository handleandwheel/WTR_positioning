/*
 * BMI088Middleware.c
 *
 *  Created on: Oct 14, 2021
 *      Author: dji
 */


#include "BMI088Middleware.h"
#include "stm32f4xx_hal.h"
#include "BMI088port.h"
#include "tim.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us)
{

	(&htim6)->Instance->CNT = (0x0000efff -us*8);//  __HAL_TIM_SET_COUNTER(&htim3,differ);

		//  HAL_TIM_Base_Start(&htim3);
		  SET_BIT(TIM6->CR1, TIM_CR1_CEN);

		//__HAL_TIM_GET_COUNTER(&htim3);
		  while(((&htim6)->Instance->CNT) < 0x0000effe)	//  READ_REG(TIM3->CNT)
		  {

		  }
		//  HAL_TIM_Base_Stop(&htim3);
		  CLEAR_BIT(TIM6->CR1, TIM_CR1_CEN);
		  /*
	uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 168;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
    */
}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
