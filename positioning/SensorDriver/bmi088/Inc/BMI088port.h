/*
 * BMI088port.h
 *
 *  Created on: 2021年10月14日
 *      Author: Liu, Yuming
 */

#ifndef BMI088_INC_BMI088PORT_H_
#define BMI088_INC_BMI088PORT_H_

#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GRYO_Pin GPIO_PIN_5
#define INT1_GRYO_GPIO_Port GPIOC
#define INT1_GRYO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

#endif /* BMI088_INC_BMI088PORT_H_ */
