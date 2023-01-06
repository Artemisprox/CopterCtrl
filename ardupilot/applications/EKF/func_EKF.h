#include "arm_math.h"
#include "drv_matrix.h"

#define EKF_sin(a)  arm_sin_f32(a)
#define EKF_cos(a)  arm_cos_f32(a)
#define EKF_tan(a)  (arm_sin_f32(a)/arm_cos_f32(a))

typedef struct
{
	float32_t pitch;
	float32_t roll;
	float32_t Vx;
	float32_t Vy;
	float32_t Kdrag;
}EKF_IMU;

typedef struct {
  float x;
  float y;
  float z;
} EKF_Accl_t;

typedef struct {
  float x;
  float y;
  float z;
} EKF_Gyro_t;

extern EKF_IMU HERO_EKF_IMU;

void EKF_Init(void);
void EKF_update(EKF_IMU* HERO_EKF_IMU,EKF_Gyro_t* Gyro_Fix ,EKF_Accl_t* Accl_Fix );
void IMU_data_update(matrix* X,EKF_IMU* IMU );

