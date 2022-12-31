#include "arm_math.h"

#define  matrix arm_matrix_instance_f32 

void  matrix_init(matrix * S, uint16_t nRows, uint16_t nCols, 	float32_t * pData);
matrix*  matrix_add(matrix* A , matrix* B , matrix* C);
matrix*  matrix_sub(matrix* A , matrix* B , matrix* C);
matrix*  matrix_multiply(matrix* A , matrix* B , matrix* C);
matrix*  matrix_inverse(matrix* A , matrix* A_1);
matrix*  matrix_trans(matrix* A , matrix* A_T);
void matrix_data_fresh(matrix* A , uint16_t Row, uint16_t Col, float32_t contant);
matrix* matrix_tri_mutiply_5(matrix* A, matrix* B, matrix* C, matrix* answer);

/*
typedef struct
{
	float32_t A11;			float32_t A12;     float32_t A13;			float32_t A14;     float32_t A15;
	float32_t A21;			float32_t A22;     float32_t A23;			float32_t A24;     float32_t A25;
	float32_t A31;			float32_t A32;     float32_t A33;			float32_t A34;     float32_t A35;
	float32_t A41;			float32_t A42;     float32_t A43;			float32_t A44;     float32_t A45;
	float32_t A51;			float32_t A52;     float32_t A53;			float32_t A54;     float32_t A55;

}matrix_5;

typedef struct
{
	uint16_t nNums;
	uint16_t nCols;
	matrix_5 matrix;
}matrix_variable;
*/