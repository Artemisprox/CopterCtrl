#ifndef DRV_IMU_H
#define DRV_IMU_H
#include <rtthread.h>
#include <rtdevice.h>

typedef enum
{
    IMU_YAW,
    IMU_PITCH,
    IMU_ROLL,
} imu_data_type_e;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} imu_euler_t;

typedef struct
{
    float gx;
    float gy;
    float gz;
} imu_gyro_t;

typedef struct
{
    imu_euler_t euler_angle;
    imu_gyro_t gyro;
}imu_t;

void Refresh_Gimbal_IMU_Data(struct rt_can_msg *msg);
void Refresh_Chassis_IMU_Data1(struct rt_can_msg *msg);
void Refresh_Chassis_IMU_Data2(struct rt_can_msg *msg);
/***
* @brief 返回云台imu数据
* @param type : 数据类型
* @retval imu_data[0]: pitch
* @retval imu_data[1]: yaw
***/
float Get_Gimbal_IMU_Data(imu_data_type_e type);

/***
* @brief 返回底盘imu数据
* @param type : 数据类型
* @retval 底盘IMU数据,范围[-180~180]
***/
float Get_Chassis_IMU_Data(imu_data_type_e type);
#endif

