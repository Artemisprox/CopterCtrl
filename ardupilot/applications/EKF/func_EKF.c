#include "func_EKF.h"
#include "drv_matrix.h"
#include "func_bmi088.h"
#include "arm_math.h"
#include <math.h>
#include "drv_IMU.h"

EKF_IMU HERO_EKF_IMU;

const float32_t g = 9.8;
const float32_t m = 1.0;
float32_t Idata[5][5] ;
matrix I;
	
float32_t Q_diag[5] = {1,1,1,1,1};
float32_t R_diag[2] = {1,1};

float32_t x_kdata[5][1] = {0}, x_k_data[5][1] = {0},  x_k_1data[5][1] = {0};
matrix x_k, x_k_, x_k_1;
float32_t 	Q_data[5][5] = {0}, R_data[2][2] = {0};
matrix Q,R;

float32_t w_data[5][5], v_data[2][2] , w_T_data[5][5], v_T_data[2][2];
matrix w,v,w_T,v_T;

float32_t Pdata[5][5] = {0} , P_data[5][5] = {0};
matrix P,P_;

float32_t A_data[5][5] = {0} , A_T_data[5][5] = {0};
matrix A,A_T;
float32_t H_data[2][5] = {0},H_T_data[5][2] = {0} ;
matrix H,H_T;
float32_t K_data[5][2] = {0} ;
matrix K;

float32_t Z_data[2][1];
matrix Z_;
float32_t Zdata[2][1];
matrix Z;

void EKF_Init(void)
{
	int i = 0;
	
	matrix_init(&I,5,5,Idata[0]);
	for(i = 0;i < 5; i++)
	{
		matrix_data_fresh(&I,i,i,1.0);
	}
	
	matrix_init(&x_k_1,5,1,x_k_1data[0]);
	
	matrix_init(&Q,5,5,Q_data[0]);
	matrix_init(&P,5,5,P_data[0]);
	for(i = 0;i < 5; i++)
	{
		matrix_data_fresh(&Q,i,i,Q_diag[i]);
	}
	
	matrix_init(&R,2,2,P_data[0]);
	for(i = 0;i < 2; i++)
	{ 
		matrix_data_fresh(&R,i,i,R_diag[i]);
	}
	
	matrix_init(&P,5,5,P_data[0]);
	
	matrix_init(&A,5,5,A_data[0]);
	matrix_init(&A_T,5,5,A_T_data[0]);
	
	matrix_init(&H,2,5,H_data[0]);
	matrix_init(&Z_,2,5,Z_data[0]);
	
	matrix_init(&w,5,5,w_data[0]);
	matrix_init(&w_T,5,5,w_T_data[0]);
	matrix_init(&v,2,2,v_data[0]);
	matrix_init(&v_T,2,2,v_T_data[0]);
	
	
	
}

void EKF_update(void)
{
/*A矩阵更新*/
	A_data[0][0] = -sinf( HERO_EKF_IMU.pitch ) * tanf( HERO_EKF_IMU.roll ) * HERO_BMI088_DEV.Gyro_Raw.z + tanf( HERO_EKF_IMU.roll )*cosf(HERO_EKF_IMU.pitch)*(HERO_BMI088_DEV.Gyro_Raw.y) + 1 ;
	A_data[0][1] = 1.0/cosf(HERO_EKF_IMU.roll)/cosf(HERO_EKF_IMU.roll)*cosf(HERO_EKF_IMU.pitch)*HERO_BMI088_DEV.Gyro_Raw.z + 1/cosf(HERO_EKF_IMU.roll)/cosf(HERO_EKF_IMU.roll)*sinf(HERO_EKF_IMU.pitch)*HERO_BMI088_DEV.Gyro_Raw.y;
	A_data[1][0] = -sinf( HERO_EKF_IMU.pitch )*HERO_BMI088_DEV.Gyro_Raw.y - cosf( HERO_EKF_IMU.pitch )*HERO_BMI088_DEV.Gyro_Raw.z;
	A_data[1][1] = 1.0;
	A_data[2][1] = -g*cosf( HERO_EKF_IMU.roll );
	A_data[2][2] = -HERO_EKF_IMU.Kdrag/m +1;
	A_data[3][0] = g*cosf(HERO_EKF_IMU.pitch)*cosf(HERO_EKF_IMU.roll);
	A_data[3][1] = -g*sinf(HERO_EKF_IMU.pitch)*sinf(HERO_EKF_IMU.roll);
	A_data[3][3] = -HERO_EKF_IMU.Kdrag/m+1;
	A_data[4][2] = -HERO_EKF_IMU.Vx/m;
	A_data[4][3] = -HERO_EKF_IMU.Vy/m;
	A_data[4][4] = 1.0;
	matrix_trans(&A,&A_T);
	
/*H矩阵更新*/
	H_data[0][2] = -HERO_EKF_IMU.Kdrag/m;
	H_data[0][4] = -HERO_EKF_IMU.Vx/m;
	H_data[1][3] = -HERO_EKF_IMU.Kdrag/m;
	H_data[1][4] = -HERO_EKF_IMU.Vy/m;
	matrix_inverse(&H,&H_T);
	
/*Xk—（先验估计）更新*/
	x_k_data[0][0] = HERO_EKF_IMU.pitch + HERO_BMI088_DEV.Gyro_Raw.x + tanf(HERO_EKF_IMU.roll)*cosf(HERO_EKF_IMU.pitch)*HERO_BMI088_DEV.Gyro_Raw.z + tanf(HERO_EKF_IMU.roll)*sinf(HERO_EKF_IMU.pitch)*HERO_BMI088_DEV.Gyro_Raw.y;
	x_k_data[1][0] = HERO_EKF_IMU.roll + cosf( HERO_EKF_IMU.pitch ) - sinf(HERO_EKF_IMU.pitch)*HERO_BMI088_DEV.Gyro_Raw.z;
	x_k_data[2][0] = -g*sinf(HERO_EKF_IMU.roll) - HERO_EKF_IMU.Kdrag*HERO_EKF_IMU.Vx/m ;
	x_k_data[3][0] = g*cosf(HERO_EKF_IMU.roll)*sinf(HERO_EKF_IMU.pitch) - HERO_EKF_IMU.Kdrag*HERO_EKF_IMU.Vy/m ;
	x_k_data[4][0] = HERO_EKF_IMU.Kdrag;

/*h(x)更新*/
	Z_data[0][0] = -x_k_data[4][0]*x_k_data[2][0]/m;
	Z_data[1][0] = -x_k_data[4][0]*x_k_data[3][0]/m;
	Zdata[0][0]  = HERO_BMI088_DEV.Accl_Raw.x;
	Zdata[1][0]	 = HERO_BMI088_DEV.Accl_Raw.y;
	
/*w噪声矩阵更新*/
	w_data[0][0] = 1;
	w_data[0][1] = tanf(HERO_EKF_IMU.roll)*sinf(HERO_EKF_IMU.pitch);
	w_data[1][1] = cosf(HERO_EKF_IMU.pitch);
	w_data[2][2] = 1;
	w_data[3][3] = 1;
	w_data[4][4] = 1;
	matrix_trans(&w,&w_T);
	
/*v噪声矩阵更新*/
	v_data[0][0] = 1;
	v_data[1][1] = 1;
	matrix_trans(&v,&v_T);

/*Pk_（先验估计）更新*/
	float32_t wQw_Tdata[5][5];
	matrix wQw_T;
	matrix_init(&wQw_T ,5,5,wQw_Tdata[0] );
	float32_t APA_Tdata[5][5];
	matrix APA_T;
	matrix_init(&APA_T ,5,5,APA_Tdata[0] );
	matrix_add(matrix_tri_mutiply_5(&A,&P,&A_T,&APA_T), matrix_tri_mutiply_5(&w,&Q,&w_T,&wQw_T), &P_);
	
/*Kalman Gain 更新*/
	float32_t HP_H_Tdata[5][5];
	matrix HP_H_T;
	matrix_init(&HP_H_T ,5,5,HP_H_Tdata[0] );
	float32_t vRv_Tdata[5][5];
	matrix vRv_T;
	matrix_init(&vRv_T ,5,5,vRv_Tdata[0] );
	float32_t Sdata[2][2];
	matrix S;
	matrix_init(&S ,2,2,Sdata[0] );
	matrix_add(matrix_tri_mutiply_5(&H,&P_,&H_T,&HP_H_T), matrix_tri_mutiply_5(&v,&R,&v_T,&vRv_T), &S);
	float32_t S_data[5][5];
	matrix S_;
	matrix_inverse(&S,&S_);
	matrix_tri_mutiply_5(&P_,&H_T,&S_,&K);
	
/*Xk后验估计更新*/
	float32_t Ydata[2][1];
	matrix Y;
	matrix_init(&Y ,2,1,Ydata[0]);
	matrix_sub(&Z,&Z_,&Y);
	
	float32_t Tdata[2][1];
	matrix T;
	matrix_init(&Y ,5,5,Ydata[0]);
	matrix_add(&x_k_,matrix_multiply(&K,&Y,&T),&x_k);
	
/*Pk后验估计*/
	float32_t T1data[2][1];
	matrix T1;
	matrix_init(&Y ,2,1,Ydata[0]);
	float32_t T2data[2][1];
	matrix T2;
	matrix_init(&Y ,5,5,Ydata[0]);
	matrix_multiply(&K,&H,&T1);
	matrix_sub(&I,&T1,&T2);
	matrix_multiply(&T2,&P_,&P);
}

void IMU_data_update(matrix* X,EKF_IMU* IMU )
{
	IMU->pitch = *(X->pData);
	IMU->roll  = *(X->pData+1);
	IMU->Vx 	 = *(X->pData+2);
	IMU->Vy    = *(X->pData+3);
	IMU->Kdrag = *(X->pData+4);
}
