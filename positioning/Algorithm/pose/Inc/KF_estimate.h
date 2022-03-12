/*
 * File name:		KF_estimate.c
 * Author:			Liu, Yuming
 * Created on:		March 9th, 2022
 * Last edit on:	March 10th, 2022
 * Description:		Run the time update and the measurement update in the KF
 */

#ifndef POSE_INC_KF_ESTIMATE_H_
#define POSE_INC_KF_ESTIMATE_H_

#include "KF_type.h"

void KF_time_update(KFStateVector *state, KFTime T, const KFVector3 *acc, const KFVector3 *gyro, KFMatrix P[5][5], const KFMatrix acc_sigma[3][3], const KFMatrix gyro_sigma[3][3]);

void KF_measurement_update(KFStateVector *state, KFTime T, const KFVector3 *mag, const KFVector2 *encoder, KFMatrix P[5][5], const KFMatrix mag_sigma[3][3], const KFMatrix encoder_sigma[2][2], float delta);

#endif /* POSE_INC_KF_ESTIMATE_H_ */
