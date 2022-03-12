/*
 * PQY13driver.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Liu, Yuming
 */

#ifndef INC_PQY13DRIVER_H_
#define INC_PQY13DRIVER_H_

#include "struct_typedef.h"

extern void PQY13_angle_read(fp32 *angle1, fp32 *angle2);

extern void PQY13_vel_read(fp32 *vel1, fp32 *vel2);

#endif /* INC_PQY13DRIVER_H_ */
