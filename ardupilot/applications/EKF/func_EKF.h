#include "arm_math.h"
#include "drv_matrix.h"

#define EKF_sin  arm_sin_f32
#define EKF_cos  arm_cos_f32


typedef struct
{
	float32_t pitch;
	float32_t roll;
	float32_t Vx;
	float32_t Vy;
	float32_t Kdrag;
}EKF_IMU;

void EKF_Init(void);
void EKF_update(void);
void IMU_data_update(matrix* X,EKF_IMU* IMU );