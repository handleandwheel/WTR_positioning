/*
 * PQY13Middleware.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Liu, Yuming
 */

#ifndef INC_PQY13MIDDLEWARE_H_
#define INC_PQY13MIDDLEWARE_H_

#include "struct_typedef.h"

void PQY13_ENC1_CSN_ENABLE(void);

void PQY13_ENC1_CSN_DISABLE(void);

void PQY13_ENC2_CSN_ENABLE(void);

void PQY13_ENC2_CSN_DISABLE(void);

void PQY13_read_single_data(uint8_t txdata, uint8_t *rxdata);

#endif /* INC_PQY13MIDDLEWARE_H_ */
