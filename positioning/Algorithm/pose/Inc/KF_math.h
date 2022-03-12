/******************************************************************************
 * File name:		KF_estimate.c
 * Author:			Liu, Yuming
 * Created on:		March 10th, 2022
 * Last edit on:	March 12nd, 2022
 * Description:		Functions of matrix calculation.
 ******************************************************************************/

#ifndef POSE_INC_KF_MATH_H_
#define POSE_INC_KF_MATH_H_

#include "KF_type.h"

/******************************************************************************
 * function: matrix multiplication
 * result: 	the result of calculation, can pass the same variable as A or B
 * A: 		left component
 * B: 		right component
 * row1: 	number of rows of A
 * col1: 	number of columns of A, should be the same as row2
 * row2: 	number of rows of B, should be the same as col1
 * col2: 	number of columns of B
 ******************************************************************************/
void KF_matrix_multiply(KFMatrix** result, const KFMatrix** A, const KFMatrix** B, int row1, int col1, int row2, int col2);

/******************************************************************************
 * function: matrix addition
 * result: 	the result of calculation, can pass the same variable as A or B
 * A: 		left component
 * B: 		right component
 * row: 	number of rows of A and B
 * col: 	number of columns of A and B
 ******************************************************************************/
void KF_matrix_add(KFMatrix** result, const KFMatrix** A, const KFMatrix** B, int row, int col);

/******************************************************************************
 * function: matrix transpose
 * result: 	the result of calculation, can pass the same variable as src
 * A: 		left component
 * B: 		right component
 * src_row: number of rows of original matrix
 * src_col: number of columns of original matrix
 ******************************************************************************/
void KF_matrix_transpose(KFMatrix** result, const KFMatrix** src, int src_row, int src_col);

/******************************************************************************
 * function: inverse the matrix
 * result: 	the inversed matrix
 * src: 	the original matrix
 * size: 	number of row or column of the matrix
 ******************************************************************************/
void KF_matrix_inverse(KFMatrix** result, const KFMatrix** src, int size);

/******************************************************************************
 * function: assign values to a matrix of the same size
 * newm: the assignee
 * oldm: the assigner
 * nrow: number of row of the new matrix
 * ncol: number of column of the old matrix
 * orow: number of rows of the matrix
 * ocol: number of columns of the matrix
 ******************************************************************************/
void KF_matrix_update(KFMatrix** newm, const KFMatrix** oldm, int nrow, int ncol, int orow, int ocol);

/******************************************************************************
 * function: assign values to a vector of the same length
 * newv: 	the assignee
 * oldv: 	the assigner
 * length: 	the length of vectors
 ******************************************************************************/
void KF_vector_update(KFVector** newv, const KFVector** oldv, int length);

/******************************************************************************
 * function: assign values to a vector of the same length
 * result: 	result of calculation
 * m:		left matrix
 * v:		right vector
 * row:		number of rows of matrix
 * col:		number of columns of matrix
 ******************************************************************************/
void KF_matrix_vector_multiply(KFVector* result, const KFMatrix** m, const KFVector* v, int row, int col);

/******************************************************************************
 * function: vector subtraction
 * result: 	the result of calculation, can pass the same variable as A or B
 * v1: 		left component
 * v2: 		right component
 * length:	length of vectors
 ******************************************************************************/
void KF_vector_subtract(KFVector* result, const KFVector* v1, const KFVector* v2, int length);

/******************************************************************************
 * function: vector addition
 * result: 	the result of calculation, can pass the same variable as A or B
 * v1: 		left component
 * v2: 		right component
 * length:	length of vectors
 ******************************************************************************/
void KF_vector_add(KFVector* result, const KFVector* v1, const KFVector* v2, int length);

#endif /* POSE_INC_KF_MATH_H_ */
