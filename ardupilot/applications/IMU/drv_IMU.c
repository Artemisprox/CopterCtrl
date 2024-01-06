#include "stm32f4xx.h"
#include "drv_IMU.h"
#include "drv_utils.h"
#include "func_TempCtr.h"
#include "drv_dataserve.h"
#include "Filter.h"


IMU_t HERO_IMU; // IMU数据,unit:m/s^2,rad/s
ATTI_t gimbal_atti;

rt_int32_t Stable_Count = 0;
int AimlossFlag = 0; // 自瞄丢失目标为0，有目标为1。有目标时需要切换成纯积分模式

rt_tick_t IMU_LastValid_tick = 0;
void IMU_transfer2gm(void);

static float lowpassfilter(float k , float now_data , float last_data)
{
	return (k*now_data + (1 - k)*last_data);
}
/***
 * @Name     gyro_read_extern
 * @brief    陀螺仪姿态解算函数调用，类似于CAN接收，用于板载陀螺仪的兼容
 * @param	  float 单位：Rad/s、Rad
 * @author   ych
 ***/
void IMU_SetData_Extern(float PitchSpe,
                        float YawSpe,
                        float RollSpe,
                        float PitchAng,
                        float YawAng,
                        float RollAng,
                        int AttiReady)
{
		static uint8_t First_flag = 1;
		static IMU_t Last_data , temp ;
		
        temp.pitch_speed = PitchSpe / 3.1415926f * 180;
		temp.yaw_speed = YawSpe / 3.1415926f * 180;
		temp.roll_speed = RollSpe / 3.1415926f * 180;
		temp.pitch = PitchAng / 3.1415926f * 180;
		temp.yaw = YawAng / 3.1415926f * 180;
		temp.roll = RollAng / 3.1415926f * 180;
		First_flag = 1;

    if(First_flag)
	{
		HERO_IMU.pitch_speed = temp.pitch_speed;
		HERO_IMU.yaw_speed = temp.yaw_speed;
		HERO_IMU.roll_speed = temp.roll_speed;
		HERO_IMU.pitch = temp.pitch;
		HERO_IMU.yaw = temp.yaw;
	    HERO_IMU.roll = temp.roll;
		HERO_IMU.atti_ready = AttiReady;
		First_flag = 0;
	}
    else
	{
	    HERO_IMU.pitch_speed = lowpassfilter(0.8f , temp.pitch_speed , Last_data.pitch_speed  );
        HERO_IMU.yaw_speed = lowpassfilter(0.8f , temp.yaw_speed , Last_data.yaw_speed  );
        HERO_IMU.roll_speed = lowpassfilter(0.8f , temp.roll_speed , Last_data.roll_speed  );
        HERO_IMU.pitch = lowpassfilter(0.8f , temp.pitch , Last_data.pitch  );
        HERO_IMU.yaw = lowpassfilter(0.8f , temp.yaw , Last_data.yaw  );
        HERO_IMU.roll = lowpassfilter(0.8f , temp.roll , Last_data.roll );
	}
		
		Last_data.pitch_speed =  HERO_IMU.pitch_speed;
		Last_data.yaw_speed = HERO_IMU.yaw_speed;
		Last_data.roll_speed = HERO_IMU.roll_speed;
		Last_data.pitch = HERO_IMU.pitch;
		Last_data.yaw = HERO_IMU.yaw;
		Last_data.roll = HERO_IMU.roll;

   // IMU_transfer2gm(); // 换算坐标系
}
/*
static ATTI_t Gimbal_Atti_Fil_Data = {0};
float jscope_yaw_bias;
static float temp_pitch_speed, temp_yaw_speed, temp_roll_speed;
//云台系(IMU)转到电机系
void IMU_transfer2gm(void)
{ // 该函数2ms运行一次
    // int dir; //融合roll角速度后的方向

    gimbal_atti.pitch = HERO_IMU.pitch - pitch_ecd_offset;
    gimbal_atti.yaw = HERO_IMU.yaw;
    gimbal_atti.roll = HERO_IMU.roll;

    // 准备开始坐标系旋转
    temp_pitch_speed = HERO_IMU.pitch_speed;
    temp_yaw_speed = HERO_IMU.yaw_speed;
    temp_roll_speed = HERO_IMU.roll_speed;
    // 沿 Pitch 轴旋转
    utils_point_rotate(&temp_roll_speed, &temp_yaw_speed, DEG2RAD_f(gimbal_atti.pitch));
    // 旋转结束, 赋值
    gimbal_atti.pitch_speed = temp_pitch_speed;
    gimbal_atti.yaw_speed = temp_yaw_speed;
    gimbal_atti.roll_speed = temp_roll_speed;

    Gimbal_Atti_Fil_Data.pitch_speed = 0.99f * Gimbal_Atti_Fil_Data.pitch_speed + 0.01f * gimbal_atti.pitch_speed;
    Gimbal_Atti_Fil_Data.yaw_speed = 0.99f * Gimbal_Atti_Fil_Data.yaw_speed + 0.01f * gimbal_atti.yaw_speed;
    Gimbal_Atti_Fil_Data.roll_speed = 0.99f * Gimbal_Atti_Fil_Data.roll_speed + 0.01f * gimbal_atti.roll_speed;

    UTILS_NAN_ZERO_F(Gimbal_Atti_Fil_Data.pitch_speed);
    UTILS_NAN_ZERO_F(Gimbal_Atti_Fil_Data.yaw_speed);
    UTILS_NAN_ZERO_F(Gimbal_Atti_Fil_Data.roll_speed);

    if (fabsf(Gimbal_Atti_Fil_Data.pitch_speed) > 45 || fabsf(Gimbal_Atti_Fil_Data.yaw_speed) > 45)
    {
        // 近期出现了大角速度情况
        Stable_Count -= (rt_int32_t)(IMU_STABLE_SET_MS / 100); // 约100ms持续高速旋转会完全清空计数值
    }
    else
    {
        Stable_Count++;
        if (Stable_Count > IMU_STABLE_SET_MS)
        {
            Stable_Count = IMU_STABLE_SET_MS;
            IMU_LastValid_tick = rt_tick_get();
        }
    }
}
*/
rt_err_t IMU_GetAttiState(void)
{
    if (Stable_Count < IMU_STABLE_SET_MS)
        return RT_ERROR;
    else
        return RT_EOK;
}

// 上电后等待陀螺仪启动完成
rt_err_t IMU_WaitForInit(void)
{
    HERO_IMU.atti_ready = 0;

    rt_thread_delay(100); // 即使陀螺仪已经正常初始化过，此时也要延时一段时间，确保不会出现刚按复位键机器人就动的情况

    while (!HERO_IMU.atti_ready)
    { // 等待陀螺仪初始化成功
        rt_thread_delay(10);
    }
    return RT_EOK;
}
