/*
 * PQY13driver.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Liu, Yuming
 */


#include "PQY13driver.h"
#include "PQY13Middleware.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "math.h"

#define PI M_PI

void PQY13_angle_read(fp32 *angle1, fp32 *angle2)
{
	uint8_t rxdata1[4];
	uint8_t rxdata2[4];

	uint16_t angle1_raw;
	uint16_t angle2_raw;

	PQY13_ENC1_CSN_ENABLE();
	PQY13_read_single_data(0x05, rxdata1);
	PQY13_ENC1_CSN_DISABLE();

	PQY13_ENC2_CSN_ENABLE();
	PQY13_read_single_data(0x05, rxdata2);
	PQY13_ENC2_CSN_DISABLE();

	angle1_raw = (((uint16_t)rxdata1[0] << 8) | (uint16_t)rxdata1[1]);
	angle2_raw = (((uint16_t)rxdata2[0] << 8) | (uint16_t)rxdata2[1]);

	*angle1 = (fp32)angle1_raw*360/65536;
	*angle2 = (fp32)angle2_raw*360/65536;
}
/*
void PQY13_vel_read(fp32 *vel1, fp32 *vel2)
{
	uint32_t t;
	uint32_t tickPerMs =  SysTick->LOAD+1;
	fp32 ang11, ang12, ang21, ang22;

	t = SysTick->VAL;
	PQY13_angle_read(&ang11, &ang12);

	t = SysTick->VAL - t;
	PQY13_angle_read(&ang21, &ang22);

	t = 1000 * t / tickPerMs;

	// delta angle may cross the zero point
	// assume that the wheels are not rolling too fast
	if(180<=ang21-ang11) 						*vel1 = ((360-(ang21-ang11))*PI*1e6/180.0)/t;
	else if(0<=ang21-ang11 && ang21-ang11<180) 	*vel1 = ((ang21-ang11)*PI*1e6/180.0)/t;
	else if(-180<=ang21-ang11 && ang21-ang11<0)	*vel1 = ((ang21-ang11)*PI*1e6/180.0)/t;
	else if(ang21-ang11<-180)					*vel1 = ((ang21-ang11)*PI*1e6/180.0)/t;
	else *vel1 = 0.0;

	if(180<=ang22-ang12) 						*vel2 = ((360-(ang22-ang12))*PI*1e6/180.0)/t;
	else if(0<=ang22-ang12 && ang22-ang12<180) 	*vel2 = ((ang22-ang12)*PI*1e6/180.0)/t;
	else if(-180<=ang22-ang12 && ang22-ang12<0)	*vel2 = ((ang22-ang12)*PI*1e6/180.0)/t;
	else if(ang22-ang12<-180)					*vel2 = ((ang22-ang12)*PI*1e6/180.0)/t;
	else *vel2 = 0.0;
}
*/
