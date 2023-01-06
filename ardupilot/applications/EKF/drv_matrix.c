#include "arm_math.h"
#include "drv_matrix.h"

void  matrix_init(matrix * S, uint16_t nRows, uint16_t nCols, 	float32_t * pData)
{
	arm_mat_init_f32(S,nRows,nCols,pData);
}

matrix*  matrix_add(matrix* A , matrix* B , matrix* C)
{
	arm_mat_add_f32(A,B,C);
	
	return C;
}

matrix*  matrix_sub(matrix* A , matrix* B , matrix* C)
{
	arm_mat_sub_f32( A ,B , C );
	
	return C;
}

matrix*  matrix_multiply(matrix* A , matrix* B , matrix* C)
{
	arm_mat_mult_f32(A,B,C);
	
	return C;
}

matrix*  matrix_inverse(matrix* A , matrix* A_1)
{
	arm_mat_inverse_f32 (A,A_1);
	
	return A_1;
}

matrix*  matrix_trans(matrix* A , matrix* A_T)
{
	arm_mat_trans_f32 (A,A_T);
	
	return A_T;
} 

void matrix_data_fresh(matrix* A , uint16_t Row, uint16_t Col, float32_t contant)
{
	float32_t** a;
	a = &(A->pData);
	if( Row <= A->numRows && Col <= A->numCols )
			a[Row-1][Col-1] = contant;
	else 
		return ;
}

matrix* matrix_tri_mutiply_5(matrix* A, matrix* B, matrix* C, matrix* answer)
{
	matrix temp;
	float32_t temp_data[5][5];
	matrix_init(&temp,5,5,temp_data[0]);
	
	matrix_multiply(A,B,&temp);
	matrix_multiply(&temp,C,answer);
	
	return answer;
}

