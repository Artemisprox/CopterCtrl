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

typedef struct 
{
    float accl;
    float last_accl;
    float accl_delta;
    float velocity;
    float last_velocity;
    float vec_delta;
    float distance;
    float last_distance;
    float dis_delta;
}vec_fusion;


#define VARIANCE_NUM 5
typedef struct 
{
    float sample[VARIANCE_NUM];
    float sample_deviation[VARIANCE_NUM];
    int const_num;
    int num;
    float average;
    float sample_sum;
    float sample_deviation_sum;
    float variance;
    int point;
    int first_flag;
}variance_data;

extern vec_fusion ins_data,flow_data;//�ں�����Դ
extern int accl_num ;
extern float accl_buffer[30];
extern float height , v_height;

extern void Velocity_estimator_init(void);
extern void Velocity_estimate(float height , float height_v , float height_observe[2] , int fuse_flag);
extern void rotation_imu_to_ground(IMU_t imu_data , float accl_data[3], float ground_accl[3]);
extern void accl_vec_estimator( IMU_t angle_data , AHRS_Accl_t accl_data ,float datarate);
extern float get_ground_accl(void);
extern void Filter_Combination(int Fil_Flag, filter_data *data,float k1,float k2,float k3);

