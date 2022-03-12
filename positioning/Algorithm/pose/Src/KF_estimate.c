/******************************************************************************
 * File name:		KF_estimate.c
 * Author:			Liu, Yuming
 * Created on:		March 10th, 2022
 * Last edit on:	March 10th, 2022
 * Description:		Run the time update and the measurement update in the KF
 ******************************************************************************/

#include "KF_estimate.h"
#include "KF_math.h"
#include "math.h"

/******************************************************************************
 * Execute the time update of KF, update the state vector and the convariance matrix
 * state: 		state vector
 * T: 			sampling time, exact time interval between two time updates
 * acc: 		acceleration meter data
 * gyro: 		gyroscope meter data
 * P: 			convariance matrix from last measurement update
 * acc_sigma: 	convariance matrix of acceleration measurement
 * gyro_sigma: 	convariance matrix of gyroscope measurement
 ******************************************************************************/
void KF_time_update(KFStateVector *state, KFTime T, const KFVector3 *acc, const KFVector3 *gyro, KFMatrix P[5][5], const KFMatrix acc_sigma[3][3], const KFMatrix gyro_sigma[3][3])
{
	KFMatrix F[5][5] = {0}; // the partial derivative matrix of state vector
	KFMatrix Q[5][5] = {0}; // the convariance matrix of process noise

	// calculate the partial derivative of state vector
	F[0][0] = 1;
	F[0][2] = -T * state->vectors.vx * sin(state->vectors.theta) - T * state->vectors.vy * cos(state->vectors.theta);
	F[0][3] = T * cos(state->vectors.theta);
	F[0][4] = -T * sin(state->vectors.theta);
	F[1][1] = 1;
	F[1][2] = T * state->vectors.vx * cos(state->vectors.theta) - T * state->vectors.vy * sin(state->vectors.theta);
	F[1][3] = T * sin(state->vectors.theta);
	F[1][4] = T * cos(state->vectors.theta);
	F[2][2] = 1;
	F[3][3] = 1;
	F[4][4] = 1;

	// calculate the convariance matrix of process noise
	Q[0][0] = (pow(T, 4)/4) * (pow(cos(state->vectors.theta), 2) * pow(acc_sigma[1][1], 2) + pow(sin(state->vectors.theta), 2) * pow(acc_sigma[2][2], 2));
	Q[0][1] = (pow(T, 4)/4) * sin(state->vectors.theta) * cos(state->vectors.theta) * (pow(acc_sigma[1][1], 2) - pow(acc_sigma[2][2], 2));
	Q[0][3] = (pow(T, 3)/2) * cos(state->vectors.theta) * pow(acc_sigma[1][1], 2);
	Q[0][4] = - (pow(T, 3)/2) * sin(state->vectors.theta) * pow(acc_sigma[2][2], 2);
	Q[1][1] = (pow(T, 4)/4) * (pow(sin(state->vectors.theta), 2) * pow(acc_sigma[1][1], 2) + pow(cos(state->vectors.theta), 2) * pow(acc_sigma[2][2], 2));
	Q[1][3] = (pow(T, 3)/2) * sin(state->vectors.theta) * pow(acc_sigma[1][1], 2);
	Q[1][4] = (pow(T, 3)/2) * cos(state->vectors.theta) * pow(acc_sigma[2][2], 2);
	Q[2][2] = pow(T, 2) * pow(gyro_sigma[3][3], 2);
	Q[3][3] = pow(T, 2) * pow(acc_sigma[1][1], 2);
	Q[4][4] = pow(T, 2) * pow(acc_sigma[2][2], 2);
	Q[1][0] = Q[0][1];
	Q[3][0] = Q[0][3];
	Q[4][0] = Q[0][4];
	Q[3][1] = Q[1][3];
	Q[4][1] = Q[1][4];

	// update convariance P
	// Note that in this way of modeling, the derivative matrix of process noise is a identity matrix,
	// thus being neglected.
	KF_matrix_multiply(P, F, P, 5, 5, 5, 5);
	KF_matrix_transpose(F, F, 5, 5);
	KF_matrix_multiply(P, P, F, 5, 5, 5, 5);
	KF_matrix_add(P, P, Q, 5, 5);

	// update state vector
	state->vectors.px = state->vectors.px + T * state->vectors.vx * cos(state->vectors.theta) - T * state->vectors.vy * sin(state->vectors.theta);
	state->vectors.px = state->vectors.px + (T*T/2) * cos(state->vectors.theta) * acc->vectors.x - (T*T/2) * sin(state->vectors.theta) * acc->vectors.y;
	state->vectors.py = state->vectors.py + T * state->vectors.vx * sin(state->vectors.theta) + T * state->vectors.vy * cos(state->vectors.theta);
	state->vectors.py = state->vectors.py + (T*T/2) * sin(state->vectors.theta) * acc->vectors.x + (T*T/2) * cos(state->vectors.theta) * acc->vectors.y;
	state->vectors.theta = state->vectors.theta + T * gyro->vectors.z;
	state->vectors.vx = state->vectors.vx + T * acc->vectors.x;
	state->vectors.vy = state->vectors.vy + T * acc->vectors.y;
}

/******************************************************************************
 * Execute the measurement update of KF, update the state vector and the convariance matrix
 * state: 		state vector
 * T: 			sampling time, exact time interval between two time updates
 * mag: 		magnetometer data
 * encoder: 	encoder data (transformed from the sensor frame to the body frame)
 * P: 			convariance matrix from last measurement update
 * mag_sigma: 	convariance matrix of magnetic field measurement
 * gyro_sigma: 	convariance matrix of encoder measurement (transformed from the sensor frame to the body frame)
 ******************************************************************************/
void KF_measurement_update(KFStateVector *state, KFTime T, const KFVector3 *mag, const KFVector2 *encoder, KFMatrix P[5][5], const KFMatrix mag_sigma[3][3], const KFMatrix encoder_sigma[2][2], float delta)
{
	KFMatrix H[4][5] = {0};
	KFMatrix S[4][4] = {0};
	KFMatrix R[4][4] = {0};
	KFMatrix K[5][4] = {0};
	KFMatrix tmp4x4[4][4] = {0};
	KFMatrix tmp4x5[4][5] = {0};
	KFMatrix tmp5x4[5][4] = {0};
	KFMatrix tmp5x5[5][5] = {0};
	KFMeasurementVector epsilon;
	KFStateVector tmp_state;

	H[0][3] = 1;
	H[1][4] = 1;
	H[2][2] = -sin(state->vectors.theta) * cos(delta);
	H[3][2] = -cos(state->vectors.theta) * cos(delta);

	R[0][0] = encoder_sigma[0][0];
	R[1][1] = encoder_sigma[1][1];
	R[2][2] = mag_sigma[0][0];
	R[3][3] = mag_sigma[1][1];

	KF_matrix_multiply(tmp4x5, H, P, 4, 5, 4, 5);
	KF_matrix_transpose(H, H, 4, 5);
	KF_matrix_multiply(S, tmp4x5, H, 4, 5, 5, 4);
	KF_matrix_add(S, S, R, 4, 4);
	KF_matrix_inverse(tmp4x4, S, 4);
	KF_matrix_multiply(K, P, H, 5, 5, 5, 4);
	KF_matrix_multiply(K, K, tmp4x4, 5, 4, 4, 4);

	epsilon.vectors.yvx = encoder->vectors.x - state->vectors.vx;
	epsilon.vectors.yvy = encoder->vectors.y - state->vectors.vy;
	epsilon.vectors.ym1 = mag->vectors.x - cos(state->vectors.theta) * cos(delta);
	epsilon.vectors.ym2 = mag->vectors.y + sin(state->vectors.theta) * cos(delta);

	KF_matrix_vector_multiply(tmp_state.vectorf, K, epsilon.vectorf, 5, 4);
	KF_vector_add(state->vectorf, state->vectorf, tmp_state.vectorf, 5);
	KF_matrix_multiply(tmp5x4, K, S, 5, 4, 4, 4);
	KF_matrix_transpose(K, K, 5, 4);
	KF_matrix_multiply(tmp5x5, tmp5x4, K, 5, 4, 4, 5);
	KF_matrix_subtract(P, P, tmp5x5, 5, 5);
}
