#include <rtthread.h>
#include "drv_IMU.h"
#include "func_SensorRAW.h"
//#include "func_sensor.h"

typedef struct 
{
    float HP_temp;
    float LP1_temp;
    float LP2_temp;
    float Raw_data;
    float Last_data;
}filter_data;

typedef struct 
{
    float v_raw;
    float v_hp;
    float v_hp_lp;
}filter_test;

extern void Velocity_estimator_init(void);
extern void Velocity_estimate(float height , float height_v , float height_observe[2] );
extern void rotation_imu_to_ground(IMU_t imu_data , float accl_data[3], float ground_accl[3]);
extern void accl_vec_estimator( IMU_t angle_data , AHRS_Accl_t accl_data );
