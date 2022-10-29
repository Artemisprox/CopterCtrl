#include "drv_MecanOmni.h"
#include "HChassis_Data.h"
#include "drv_utils.h"
#include "drv_wheel.h"


/**
 * @brief   麦轮/全向底盘运动解算
 * @param   chs.vel     底盘xy方向速度大小,单位mm/s
 * @param   chs.angvel  �?�?角速度,单位0.1°/s,底盘逆时针旋�?为�??
 * @param   output      输出的底盘每�?电机的转速�?�定�?
 * @return  None
 * @author  lfp
 */
void MecanOmni_Resolve(MecanOmni_t chs, float output[])
{
	//麦轮运动解算
#ifdef MECANUM_WHEEL
    //*MY_PI/1800.0为单位转换系数，*(VEHICLE_WIDTH + VEHICLE_LONG)/2.0f将�?�速度�?化为�?毂的线速度
    const float coefficient = PI / 1800.0f * (float)(VEHICLE_WIDTH + VEHICLE_LONG) / 2.0f;

	//v1~4单位rad/s
    output[0] = chs.vel.y - chs.vel.x + chs.angvel * coefficient;
    output[1] = chs.vel.y + chs.vel.x - chs.angvel * coefficient;
    output[2] = chs.vel.y - chs.vel.x - chs.angvel * coefficient;
    output[3] = chs.vel.y + chs.vel.x + chs.angvel * coefficient;
#endif
    
	//全向�?运动解算
#ifdef OMNI_WHEEL
    //*Lx_PI/1800.0为单位转换系数，*(VEHICLE_DIAMETER/2.0)将�?�速度�?化为线速度
    const float coefficient = Lx_PI / 1800.0f * (float)(VEHICLE_DIAMETER / 2.0f);

	//v1~4单位rad/s
    output[0] = chs.vel.y + chs.angvel * coefficient;
    output[1] = chs.vel.x - chs.angvel * coefficient;
    output[2] = chs.vel.y - chs.angvel * coefficient;
    output[3] = chs.vel.x + chs.angvel * coefficient;
#endif
}

