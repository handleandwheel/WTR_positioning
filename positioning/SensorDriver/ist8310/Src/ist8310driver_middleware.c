
  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       IST8310driver_middleware.c/h
  * @brief      the file provide I2C write/read function, as the middleware of IST8310.
  * @note       IST8310 only support I2C.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "ist8310driver_middleWare.h"
#include "ist8310port.h"
#include "tim.h"

extern I2C_HandleTypeDef hi2c3;

/**
  * @brief          initialize ist8310 gpio.
  * @param[in]      none
  * @retval         none
  */
void ist8310_GPIO_init(void)
{

}

/**
  * @brief          initialize ist8310 communication interface
  * @param[in]      none
  * @retval         none
  */
void ist8310_com_init(void)
{
}

/**
  * @brief          read a byte of ist8310 by i2c
  * @param[in]      register address
  * @retval         value of the register
  */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,&res,1,10);
    return res;
}


/**
  * @brief          write a byte of ist8310 by i2c
  * @param[in]      register address
  * @param[in]      write value
  * @retval         value of the register
  */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,&data,1,10);
}

/**
  * @brief          read multiple byte of ist8310 by i2c
  * @param[in]      register start address
  * @param[out]     read buffer
  * @param[in]      Size Amount of data to be read
  * @retval         none
  */
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,buf,len,10);
}


/**
  * @brief          write multiple byte of ist8310 by i2c
  * @param[in]      register address
  * @param[out]     write buffer
  * @param[in]      Size Amount of data to be sent
  * @retval         none
  */
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,data,len,10);
}

/**
  * @brief          delay x millisecond
  * @param[in]      ms: ms millisecond
  * @retval         none
  */
void ist8310_delay_ms(uint16_t ms)
{
	while(ms--)
	{
		ist8310_delay_us(1000);
	}
}


/**
  * @brief          delay x microsecond
  * @param[in]      us: us microsecond
  * @retval         none
  */
void ist8310_delay_us(uint16_t us)
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
    uint32_t told = 0, tnow = 0, tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 72;
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


/**
  * @brief          set the RSTN PIN to 1
  * @param[in]      none
  * @retval         none
  */
void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
}


/**
  * @brief          set the RSTN PIN to 0
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
}
