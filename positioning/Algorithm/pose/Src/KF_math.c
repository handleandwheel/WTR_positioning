/******************************************************************************
 * File name:		KF_estimate.c
 * Author:			Liu, Yuming
 * Created on:		March 10th, 2022
 * Last edit on:	March 12nd, 2022
 * Description:		Functions of matrix calculation.
 * In C, we cannot define a array with variables being dimensions,
 * and in this KF, no matrix is larger than 5*5. Therefore, all the
 * tmp matrix in this file are in the size of 5*5.
 ******************************************************************************/

#include "KF_math.h"
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
void KF_matrix_multiply(KFMatrix** result, const KFMatrix** A, const KFMatrix** B, int row1, int col1, int row2, int col2)
{
	int i, j, m;
	KFMatrix tmp[5][5] = {0};
	for(i = 0; i < row1; i++)
	{
		for(j = 0; j < col2; j++)
		{
			for(m = 0; m < col1; m++)
			{
                tmp[i][j] += (*((KFMatrix*)A + i*col1 + m)) * (*((KFMatrix*)B + m*col2 + j));
			}
		}
	}
	KF_matrix_update(result, tmp, row1, col2, 5, 5);
}

/******************************************************************************
 * function: matrix addition
 * result: 	the result of calculation, can pass the same variable as A or B
 * A: 		left component
 * B: 		right component
 * row: 	number of rows of A and B
 * col: 	number of columns of A and B
 ******************************************************************************/
void KF_matrix_add(KFMatrix** result, const KFMatrix** A, const KFMatrix** B, int row, int col)
{
	int i, j;
	for(i = 0; i < row; i++)
	{
		for(j = 0; j < col; j++)
		{
			*((KFMatrix*)result + i*col + j) = *((KFMatrix*)A + i*col + j) + *((KFMatrix*)B + i*col + j);
		}
	}
}

/******************************************************************************
 * function: matrix subtraction
 * result: 	the result of calculation, can pass the same variable as A or B
 * A: 		left component
 * B: 		right component
 * row: 	number of rows of A and B
 * col: 	number of columns of A and B
 ******************************************************************************/
void KF_matrix_subtract(KFMatrix** result, const KFMatrix** A, const KFMatrix** B, int row, int col)
{
	int i, j;
	for(i = 0; i < row; i++)
	{
		for(j = 0; j < col; j++)
		{
			*((KFMatrix*)result + i*col + j) = *((KFMatrix*)A + i*col + j) + *((KFMatrix*)B + i*col + j);
		}
	}
}

/******************************************************************************
 * function: matrix transpose
 * result: 	the result of calculation, can pass the same variable as src
 * A: 		left component
 * B: 		right component
 * src_row: number of rows of original matrix
 * src_col: number of columns of original matrix
 ******************************************************************************/
void KF_matrix_transpose(KFMatrix** result, const KFMatrix** src, int src_row, int src_col)
{
	int i, j;
	KFMatrix tmp[5][5] = {0};
	for(i = 0; i < src_col; i++)
	{
		for(j = 0; j < src_row; j++)
		{
			tmp[i][j] = *((KFMatrix*)src + j*src_col + i);
		}
	}
	KF_matrix_update(result, tmp, src_col, src_row, 5, 5);
}

/******************************************************************************
 * function: inverse the matrix
 * result: 	the inversed matrix
 * src: 	the original matrix
 * size: 	number of row or column of the matrix
 ******************************************************************************/
void KF_matrix_inverse(KFMatrix** result, const KFMatrix** src, int size)
{
	KFMatrix tmp_src[5][5] = {0};
	KFMatrix tmp_result[5][5] = {0};
	int i, j, k;
	float main_val;
	float param;

	// copy src matrix to tmp_src matrix
	for(i = 0; i < size; i++)
	{
		for(j = 0; j < size; j++)
		{
			tmp_src[i][j] = *((KFMatrix*)src + i*size + j);
		}
	}

	// initialize augmented matrix
	for(i = 0; i < size; i++)
	{
		tmp_result[i][i] = 1;
	}

	// Gaussian elimination
	for(i = 0; i < size; i++)
	{
		main_val = tmp_src[i][i];
		if(fabs(main_val) < 1e-7) break;  // if the next main value is zero
		for(j = 0; j < size; j++)
        {
            tmp_src[i][j] = tmp_src[i][j] / main_val;
            tmp_result[i][j] = tmp_result[i][j] / main_val;
        }
		for(j = 0; j < size; j++)
		{
			if(i != j)
            {
                param = tmp_src[j][i];
                for(k = 0; k < size; k++)
                {
                    tmp_src[j][k] = tmp_src[j][k] - tmp_src[i][k] * param;
                    tmp_result[j][k] = tmp_result[j][k] - tmp_result[i][k] * param;
                }
            }
		}
	}
	KF_matrix_update(result, tmp_result, size, size, 5, 5);
}

/******************************************************************************
 * function: assign values to a matrix of the same size
 * newm: the assignee
 * oldm: the assigner
 * nrow: number of row of the new matrix
 * ncol: number of column of the old matrix
 * orow: number of rows of the matrix
 * ocol: number of columns of the matrix
 ******************************************************************************/
void KF_matrix_update(KFMatrix** newm, const KFMatrix** oldm, int nrow, int ncol, int orow, int ocol)
{
	int i, j;
	for(i = 0; i < nrow; i++)
	{
		for(j = 0; j < ncol; j++)
		{
			*((KFMatrix*)newm + i*ncol + j) = *((KFMatrix*)oldm + i*ocol + j);
		}
	}
}

/******************************************************************************
 * function: assign values to a vector of the same length
 * newv: 	the assignee
 * oldv: 	the assigner
 * length: 	the length of vectors
 ******************************************************************************/
void KF_vector_update(KFVector** newv, const KFVector** oldv, int length)
{
    int i;
    for(i = 0; i < length; i++)
    {
        *((KFMatrix*)newv + i) = *((KFMatrix*)oldv + i);
    }
}

/******************************************************************************
 * function: assign values to a vector of the same length
 * result: 	result of calculation
 * m:		left matrix
 * v:		right vector
 * row:		number of rows of matrix
 * col:		number of columns of matrix
 ******************************************************************************/
void KF_matrix_vector_multiply(KFVector* result, const KFMatrix** m, const KFVector* v, int row, int col)
{
	int i, j;
	KFVector tmp[5] = {0};
	for(i = 0; i < row; i++)
	{
		for(j = 0; j < col; j++)
		{
			tmp[i] += *((KFMatrix*)m + i*col + j) * *((KFVector*)v + j);
		}
	}
	KF_vector_update(result, tmp, row);
}

/******************************************************************************
 * function: vector subtraction
 * result: 	the result of calculation, can pass the same variable as A or B
 * v1: 		left component
 * v2: 		right component
 * length:	length of vectors
 ******************************************************************************/
void KF_vector_subtract(KFVector* result, const KFVector* v1, const KFVector* v2, int length)
{
	int i;
	KFVector tmp[5] = {0};
	for(i = 0; i < length; i++)
	{
		tmp[i] = *((KFVector*)v1 + i) - *((KFVector*)v2 + i);
	}
	KF_vector_update(result, tmp, length);
}

/******************************************************************************
 * function: vector addition
 * result: 	the result of calculation, can pass the same variable as A or B
 * v1: 		left component
 * v2: 		right component
 * length:	length of vectors
 ******************************************************************************/
void KF_vector_add(KFVector* result, const KFVector* v1, const KFVector* v2, int length)
{
	int i;
	KFVector tmp[5] = {0};
	for(i = 0; i < length; i++)
	{
		tmp[i] = *((KFVector*)v1 + i) + *((KFVector*)v2 + i);
	}
	KF_vector_update(result, tmp, length);
}
