#include "arm_math.h"
#include "drv_matrix.h"

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