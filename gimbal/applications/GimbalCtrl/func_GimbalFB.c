#include "func_GimbalFB.h"
#include "robodata.h"
#include "drv_utils.h"

FBData_S GimbalFB = {FB_NONE}; // 反馈量相关内容

// 使用云台编码器计算当前云台相对于底盘的姿态
static float Pitch_ENCD, Yaw_ENCD; // Pitch轴编码器数据换算数据零位和单位 便于JSCOPE观察
static void Get_ENCD_ATTI(AttitudeData_Type *Gimbal_ENCD_Data_Get)
{
    Pitch_ENCD = Motor_Get_DeltaAngle(Motor_Read_NowEncoder(&Pitch), PITCH_ZERO_ANGLE, 8192.0f) * 360.0f / 8192;
    Yaw_ENCD = Motor_Get_DeltaAngle(Motor_Read_NowEncoder(&Yaw), YAW_ZERO_ANGLE, 8192.0f) * 360.0f / 8192;

    if (Pitch_ENCD > 90.0f)
        Pitch_ENCD = 90.0f;
    else if (Pitch_ENCD < -90.0f)
        Pitch_ENCD = -90.0f;

    utils_norm_circle_number(&Yaw_ENCD, -180.f, 360.f);

    Gimbal_ENCD_Data_Get->Pitch = Pitch_ENCD;
    Gimbal_ENCD_Data_Get->Yaw = Yaw_ENCD;
}

// 刷新IMU姿态数据
static void Get_IMU_ATTI(AttitudeData_Type *Gimbal_IMU_Data_Get)
{ // 直接记录当前IMU数值
    Gimbal_IMU_Data_Get->Pitch = gimbal_atti.pitch;
    Gimbal_IMU_Data_Get->Yaw = gimbal_atti.yaw;
}

/* 关于切换过程：
 *  从陀螺仪切换至编码器：编码器反馈量会按照当前陀螺仪数值进行相应偏置处理，数据没有跳变
 *  从编码器切换至陀螺仪：陀螺仪数据一般与编码器+偏置后的数据有区别，此时反馈数据会出现跳变，需要通过修改设定值实现无振动切换
 */

// 刷新：云台反馈量计算结构体 GimbalFB
static GimbalFBSelection_E Pitch_FBS_Old = FB_IMU, Yaw_FBS_Old = FB_IMU;
void Gimbal_PID_FB_Fresh(void)
{
    float PitchTemp, YawTemp;
    // 刷新结构体基础数据
    Get_ENCD_ATTI(&GimbalFB.ENCD_ATTI);
    Get_IMU_ATTI(&GimbalFB.IMU_ATTI);
    GimbalFB.DeltaAtti.Pitch = GimbalFB.ENCD_ATTI.Pitch + GimbalFB.ENCD_BIAS.Pitch - GimbalFB.IMU_ATTI.Pitch; // 更新角度差
    GimbalFB.DeltaAtti.Yaw = GimbalFB.ENCD_ATTI.Yaw + GimbalFB.ENCD_BIAS.Yaw - GimbalFB.IMU_ATTI.Yaw;

    // 计算输出数据
    PitchTemp = GimbalFB.FB_This.Pitch;
    YawTemp = GimbalFB.FB_This.Yaw;
    if (GimbalFB.FBS_Now_Pitch == FB_ENCD)
    { // 本次使用编码器数据
        if (Pitch_FBS_Old != FB_ENCD)
            PitchTemp = GimbalFB.ENCD_ATTI.Pitch + GimbalFB.ENCD_BIAS.Pitch;
        else
            PitchTemp = PitchTemp * 0.001f + 0.999f * (GimbalFB.ENCD_ATTI.Pitch + GimbalFB.ENCD_BIAS.Pitch); // 输出编码器数值时添加切换数据源时记录的偏置量
    }
    else
    { // 如果没有初始化 或 需要使用陀螺仪数据 进行闭环，则直接使用陀螺仪数据进行输出，同时更新ENCD_BIAS
        GimbalFB.ENCD_BIAS.Pitch = gimbal_atti.pitch - GimbalFB.ENCD_ATTI.Pitch;
        PitchTemp = gimbal_atti.pitch;
    }

    if (GimbalFB.FBS_Now_Yaw == FB_ENCD)
    {
        if (Yaw_FBS_Old != FB_ENCD)
            YawTemp = GimbalFB.ENCD_ATTI.Yaw + GimbalFB.ENCD_BIAS.Yaw;
        else
        {
            // 输出编码器数值时添加切换数据源时记录的偏置量
            YawTemp = GimbalFB.FB_This.Yaw + utils_angle_difference(GimbalFB.ENCD_ATTI.Yaw + GimbalFB.ENCD_BIAS.Yaw, GimbalFB.FB_This.Yaw) * 0.5f; // 滞后滤波
            // 重新转换回 [-180°,180°]
            utils_norm_circle_number(&YawTemp, -180.f, 360.f);
        }
    }
    else
    {
        GimbalFB.ENCD_BIAS.Yaw = gimbal_atti.yaw - GimbalFB.ENCD_ATTI.Yaw;
        YawTemp = gimbal_atti.yaw;
    }

    // 数据限幅、跨圈修正
    if (PitchTemp > 90.0f)
        PitchTemp = 90.0f;
    else if (PitchTemp < -90.0f)
        PitchTemp = -90.0f;

    utils_norm_circle_number(&YawTemp, -180.f, 360.f);

    // 输出数据
    GimbalFB.FB_This.Pitch = PitchTemp;
    GimbalFB.FB_This.Yaw = YawTemp;
    Pitch_FBS_Old = GimbalFB.FBS_Now_Pitch;
    Yaw_FBS_Old = GimbalFB.FBS_Now_Yaw;
}

// 获取当前云台角度反馈量
void Gimbal_FB_Get(AttitudeData_Type *FB_Atti)
{ // 输出反馈量
    FB_Atti->Pitch = GimbalFB.FB_This.Pitch;
    FB_Atti->Yaw = GimbalFB.FB_This.Yaw;
}

// 设定现在使用的的反馈数据源
void GimbalPitch_FB_Select_Set(GimbalFBSelection_E FB_Set)
{
    GimbalFB.FBS_Now_Pitch = FB_Set; // 直接更新设定的数据源
}
// 设定现在使用的的反馈数据源
void GimbalYaw_FB_Select_Set(GimbalFBSelection_E FB_Set)
{
    GimbalFB.FBS_Now_Yaw = FB_Set; // 直接更新设定的数据源
}
// 读取现在使用的的反馈数据源
GimbalFBSelection_E GimbalYaw_FB_Select_Get(void)
{
    return GimbalFB.FBS_Now_Yaw;
}
// 读取现在使用的的反馈数据源
GimbalFBSelection_E GimbalPitch_FB_Select_Get(void)
{
    return GimbalFB.FBS_Now_Pitch;
}

// 初始化云台控制反馈信息
void GimbalFB_Init(void)
{
    // 云台反馈量默认使用IMU数据
    GimbalFB.FBS_Now_Pitch = FB_IMU;
    GimbalFB.FBS_Now_Yaw = FB_IMU;

    Gimbal_PID_FB_Fresh(); // 立即刷新一次数据 要求等待电机上电以及相关初始化完成后执行
}
