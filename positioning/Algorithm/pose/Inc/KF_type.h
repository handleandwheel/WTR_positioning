/******************************************************************************
 * File name: 		KF_type.h
 * Author: 			Liu, Yuming
 * Date: 			March 9th, 2022
 * Description: 	Some types used in the KF algorithm.
 ******************************************************************************/

#ifndef POSE_INC_KF_TYPE_H_
#define POSE_INC_KF_TYPE_H_

/* redefine float type to be column vector */
typedef float KFVector;

/* 3-dimension vector */
typedef union KFVector3
{
	KFVector vectorf[3];
	struct
	{
		float x;
		float y;
		float z;
	} vectors;
} KFVector3;

/* 2-dimension vector */
typedef struct KFVector2
{
	KFVector vectorf[2];
	struct
	{
		float x;
		float y;
	} vectors;
} KFVector2;

/******************************************************************************
 * The state vector
 * px: 		position of axis x with respect to the initial position in the navigation frame
 * py: 		position of axis y with respect to the initial position in the navigation frame
 * theta: 	angle of plane rotation, positive direction is up (same value in the body frame and the navigation frame)
 * vx: 		linear velocity in x-axis direction in body frame
 * vy: 		linear velocity in y-axis direction in body frame
 ******************************************************************************/
typedef struct KFStateVector
{
	KFVector vectorf[5];
	struct
	{
		float px;
		float py;
		float theta;
		float vx;
		float vy;
	} vectors;
} KFStateVector;

/******************************************************************************
 * The measurement vector
 * yvx: 	measurement of the linear velocity in x-axis in body frame
 * yvy: 	measurement of the linear velocity in y-axis in body frame
 * ym1: 	first component of magnetometer
 * ym2: 	second component of magnetometer
 * (this is an estimation on the plane, only these 2 components are useful)
 ******************************************************************************/
typedef struct KFMeasurementVector
{
	KFVector vectorf[4];
	struct
	{
		float yvx;
		float yvy;
		float ym1;
		float ym2;
	} vectors;
} KFMeasurementVector;

/* redefine float type to be matrix */
typedef float KFMatrix;

/* redefine float type to be time */
typedef float KFTime;

#endif /* POSE_INC_KF_ESTIMATE_H_ */


