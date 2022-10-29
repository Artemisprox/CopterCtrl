#include "drv_IMU.h"
#include "Can_Receive.h"

static imu_t HERO_Gimbal_IMU; // 云台向左/上旋转时对应轴的数据增大
static imu_t HERO_Chassis_IMU;

void Refresh_Gimbal_IMU_Data(struct rt_can_msg *msg)
{
    HERO_Gimbal_IMU.euler_angle.pitch = *((float *)&msg->data[0]);
    HERO_Gimbal_IMU.euler_angle.yaw = *((float *)&msg->data[4]);
}

void Refresh_Chassis_IMU_Data1(struct rt_can_msg *msg)
{
    HERO_Chassis_IMU.euler_angle.pitch = ((rt_int16_t)(msg->data[0] << 8 | msg->data[1])) / 100.0f; // pitch    -90----90,在"建议安装坐标系"下,面对枪口,上正下负,单位:°
    HERO_Chassis_IMU.euler_angle.yaw = ((rt_int16_t)(msg->data[2] << 8 | msg->data[3])) / 100.0f; // yaw      -180----180,在"建议安装坐标系"下,面对枪口,左负右正
    HERO_Chassis_IMU.euler_angle.roll = ((rt_int16_t)(msg->data[4] << 8 | msg->data[5])) / 100.0f;  // roll     -180----180,在"建议安装坐标系"下,面对枪口,逆时针正,顺时针负
}

void Refresh_Chassis_IMU_Data2(struct rt_can_msg *msg)
{
    HERO_Chassis_IMU.gyro.gy = ((rt_int16_t)(msg->data[0] << 8 | msg->data[1])) / 100.0f; // pitch    -90----90,在"建议安装坐标系"下,面对枪口,上正下负,单位:°
    HERO_Chassis_IMU.gyro.gz = ((rt_int16_t)(msg->data[2] << 8 | msg->data[3])) / 100.0f; // yaw      -180----180,在"建议安装坐标系"下,面对枪口,左负右正
    HERO_Chassis_IMU.gyro.gx = ((rt_int16_t)(msg->data[4] << 8 | msg->data[5])) / 100.0f; // roll     -180----180,在"建议安装坐标系"下,面对枪口,逆时针正,顺时针负
}

/***
* @brief 返回云台imu数据
* @param type : 数据类型
* @retval 云台IMU数据,范围[-180~180]
***/
float Get_Gimbal_IMU_Data(imu_data_type_e type)
{
    switch (type)
    {
        case IMU_YAW:
            return HERO_Gimbal_IMU.euler_angle.yaw;
        case IMU_PITCH:
            return HERO_Gimbal_IMU.euler_angle.pitch;
    }
    return 0;
}

/***
* @brief 返回底盘imu数据
* @param type : 数据类型
* @retval 底盘IMU数据,范围[-180~180]
***/
float Get_Chassis_IMU_Data(imu_data_type_e type)
{
    switch (type)
    {
    case IMU_YAW:
        return HERO_Chassis_IMU.euler_angle.yaw;
    case IMU_PITCH:
        return HERO_Chassis_IMU.euler_angle.pitch;
    case IMU_ROLL:
        return HERO_Chassis_IMU.euler_angle.roll;
    }
    return 0;
}
