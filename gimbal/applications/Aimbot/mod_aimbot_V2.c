#include "drv_Aimbot_Public.h"
#include "mod_aimbot_V2.h"
#include "mod_gimbal.h"
#include "drv_thread.h"
#include "drv_GimbalPublic.h"
#include "func_gun.h"
#include "drv_utils.h"
#include "drv_ExactSmooth.h"

static float Gimbal_Pitch_UpLim, Gimbal_Pitch_DownLim;       // 记录当前云台能达到的角度限幅数据
ExactSmth_CTRL_S Smooth_VisualPitchSet, Smooth_VisualYawSet; // 用于视觉设定值的平滑滤波

struct Accuracy_Check_Module
{
    float PitchTolerance; // Pitch 轴的容许误差
    float YawTolerance;   // Yaw 轴的容许误差
    float PitchErr;       // Pitch 轴控制误差
    float YawErr;         // Yaw 轴控制误差
} Accuracy_Check_s;       // 精度检查模块

//刷新云台限幅设定值
//输入单位：度
void Refresh_Gimbal_Lim(float Up, float Down)
{
    if (Up < Down)
    {              //检查角度是否正常
        Down = Up; //优先抬头
    }
    Gimbal_Pitch_UpLim = Up;
    Gimbal_Pitch_DownLim = Down;
}

//刷新弹速设定值
//输入单位：m/s
void Refresh_Muzzle_V(float V_New)
{
    Muzzle_V_REM = V_New; // 修改弹速后，会在AimbotCAN文件中的下一次对时通信时应用给视觉端
}

// 切换自瞄模式函数
void Refresh_VisualMode(drv_VisualMode_e Mode_Set)
{
    Visual_Mode_Set = Mode_Set;
    if ((int)VisualMode_FB != (int)Visual_Mode_Set)
    { // 切换至不同的自瞄模式后，清空已有的设定值数据
        GimbalSet_Receive[0].State = RT_ERROR;
        GimbalSet_Receive[1].State = RT_ERROR;
    }
}

static int ShortTime_GunSet_AimbotShoot_Count = 0;
// 短时性精度检查: 在比较短的时间内做滞回比较, 但是对精度的要求相对比较苛刻
static int ShortTime_Accuracy_Check(struct Accuracy_Check_Module *Accuracy_Check_Struct)
{
    if ((fabsf(Accuracy_Check_Struct->PitchErr) < Accuracy_Check_Struct->PitchTolerance) &&
        (fabsf(Accuracy_Check_Struct->YawErr) < Accuracy_Check_Struct->YawTolerance))
    {
        // 本次精度检查通过
        if (ShortTime_GunSet_AimbotShoot_Count < 40)
            ++ShortTime_GunSet_AimbotShoot_Count;
    }
    else
    {
        // 本次精度检查未通过
        if (ShortTime_GunSet_AimbotShoot_Count > 5)
            ShortTime_GunSet_AimbotShoot_Count -= 5;
        else
            ShortTime_GunSet_AimbotShoot_Count = 0;
    }
    // 检查最终精度结果
    if (ShortTime_GunSet_AimbotShoot_Count < 30)
        return 0;
    else
        return 1;
}

static int LongTime_GunSet_AimbotShoot_Count = 0;
static float LongTime_Filted_PitchErr = 0.f;
static float LongTime_Filted_YawErr = 0.f;
// 长时性精度检查: 在比较长的时间内对控制误差做大的滞回比较, 但是在判断时需要严格连续时间内控制精度达标
static int LongTime_Accuracy_Check(struct Accuracy_Check_Module *Accuracy_Check_Struct, const int LastResult)
{
    // 如果上一次总精度检查通过, 本次计数清零
    if (LastResult)
        LongTime_GunSet_AimbotShoot_Count = 0;
    LongTime_Filted_PitchErr = UTILS_LP_FAST(LongTime_Filted_PitchErr, Accuracy_Check_Struct->PitchErr, 0.9f);
    LongTime_Filted_YawErr = UTILS_LP_FAST(LongTime_Filted_YawErr, Accuracy_Check_Struct->YawErr, 0.9f);
    if ((fabsf(LongTime_Filted_PitchErr) < Accuracy_Check_Struct->PitchTolerance) &&
        (fabsf(LongTime_Filted_YawErr) < Accuracy_Check_Struct->YawTolerance))
    {
        // 本次精度检查通过
        if (LongTime_GunSet_AimbotShoot_Count < 200)
            ++LongTime_GunSet_AimbotShoot_Count;
    }
    else
    {
        // 本次精度检查未通过
        LongTime_GunSet_AimbotShoot_Count = 0;
    }
    // 检查最终精度结果
    if (LongTime_GunSet_AimbotShoot_Count < 150)
        return 0;
    else
    {
        LongTime_GunSet_AimbotShoot_Count = 0;
        return 1;
    }
}

/**
 * @brief 重启视觉给定的设定角度的滤波器
 * @author fwlh
 * @param  NewPitch         重启以后的 Pitch 设定值
 * @param  NewYaw           重启以后的 Yaw 设定值
 */
void Smooth_Restart_VisualSet(float NewPitch, float NewYaw)
{
    Smooth_SetData_Restart(&Smooth_VisualPitchSet, NewPitch);
    Smooth_SetData_Restart(&Smooth_VisualYawSet, NewYaw);
}

static Gimbal_SetReceive_Type GimbalSet_GetFromVisual;
static rt_tick_t Last_Predicted_Tick;           // 上一次收到有效数据时的预测时间点
static drv_VisualMode_e Visual_Mode_Set_Last;   // 上一次的视觉设定工作模式
static uint8_t LongTime_Accuracy_Check_Result;  // 长时间精度检查的结果
static uint8_t ShortTime_Accuracy_Check_Result; // 短时间精度检查的结果
rt_int32_t DeltaTick;
// 输入输出角度单位：°
// 输入输出零位：与IMU数据零位一致
// 获取所需的云台角度设定值函数
void Aimbot_Get_GimbalSet(Gimbal_SetCal_Type *Gimbal_SetData_Out)
{
    char ReadValid_Rem;
    rt_tick_t TickNow = rt_tick_get(); // 获取并保存当前的Tick

    AttitudeData_Type GimbalSet_Atti_Temp; // 该变量包含较多中间过程, 不建议使用 jscope 观察

    // 检查自瞄模式是否正常，检查是否瞄准到了目标
    if (((int)Visual_Mode_Set != (int)VisualMode_FB) || (VisualFlag_TargetFound == 0))
    {
        // 当前视觉模式有问题，或没有识别到目标，则不使用当前的设定值
        // 当前获取不到可用的自瞄数据，则直接返回错误值
        Gimbal_SetData_Out->State = RT_ERROR;
        return;
    }

ReadAgain:
    ReadValid_Rem = Gimbal_Set_Cal_READ_Valid;
    GimbalSet_GetFromVisual.GimbalSet_Angle.Pitch = GimbalSet_Receive[ReadValid_Rem].GimbalSet_Angle.Pitch;
    GimbalSet_GetFromVisual.GimbalSet_Angle.Yaw = GimbalSet_Receive[ReadValid_Rem].GimbalSet_Angle.Yaw;
    GimbalSet_GetFromVisual.GimbalSet_Speed.Pitch = GimbalSet_Receive[ReadValid_Rem].GimbalSet_Speed.Pitch;
    GimbalSet_GetFromVisual.GimbalSet_Speed.Yaw = GimbalSet_Receive[ReadValid_Rem].GimbalSet_Speed.Yaw;
    GimbalSet_GetFromVisual.PredictedTime = GimbalSet_Receive[ReadValid_Rem].PredictedTime;
    GimbalSet_GetFromVisual.State = GimbalSet_Receive[ReadValid_Rem].State;
    if (ReadValid_Rem != Gimbal_Set_Cal_READ_Valid)
        /* 中途出现了视觉数据更新，此时需要重新读取 */
        goto ReadAgain;

    // 设定值读取完成, 先判断有效性
    DeltaTick = TickNow - GimbalSet_GetFromVisual.PredictedTime; // 计算已知设定值的时刻到现在的时间差
    if ((GimbalSet_GetFromVisual.State != RT_EOK) || (DeltaTick > 50))
    {
        // 当前获取不到可用的自瞄数据，则直接返回错误值
        Gimbal_SetData_Out->State = RT_ERROR;
        VisualFlag_TargetFound = 0; // 强行标记为丢失目标状态
        return;
    }

    // 数据有效，正常换算设定值
    GimbalSet_Atti_Temp.Pitch = GimbalSet_GetFromVisual.GimbalSet_Angle.Pitch + (GimbalSet_GetFromVisual.GimbalSet_Speed.Pitch / 1000.0f) * DeltaTick;
    // 判断Pitch轴是否能达到指定设定值
    if ((GimbalSet_Atti_Temp.Pitch > Gimbal_Pitch_UpLim) || (GimbalSet_Atti_Temp.Pitch < Gimbal_Pitch_DownLim))
    {
        // 当前获取不到可用的自瞄数据，则直接返回错误值
        Gimbal_SetData_Out->State = RT_ERROR;
        return;
    }

    GimbalSet_Atti_Temp.Yaw = GimbalSet_GetFromVisual.GimbalSet_Angle.Yaw + (GimbalSet_GetFromVisual.GimbalSet_Speed.Yaw / 1000.0f) * DeltaTick;

    // Yaw跨圈处理
    utils_norm_angle(&GimbalSet_Atti_Temp.Yaw);
    // 精度检查
    if (VisualFlag_TargetFound == 1)
    {
        // 有锁定目标，需要计算云台精度是否达标
        if (VisualFlag_Fire || (VisualMode_FB == VISUAL_MODE_AIMBUFF_CONST_SPEED) || (VisualMode_FB == VISUAL_MODE_AIMBUFF_VARY_SPEED))
        {
            Accuracy_Check_s.PitchErr = CtrlErr_Pitch;
            Accuracy_Check_s.YawErr = CtrlErr_Yaw;
            Accuracy_Check_s.PitchTolerance = GimbalTolerance_Pitch;
            Accuracy_Check_s.YawTolerance = GimbalTolerance_Yaw;
            // 两种精度判断逻辑取交集
            LongTime_Accuracy_Check_Result = LongTime_Accuracy_Check(&Accuracy_Check_s, GunSet_AimbotShootFlag);
            ShortTime_Accuracy_Check_Result = ShortTime_Accuracy_Check(&Accuracy_Check_s);
            GunSet_AimbotShootFlag = LongTime_Accuracy_Check_Result || ShortTime_Accuracy_Check_Result;
        }
    }
    else
    {
        // 没有锁定目标，显然不应发弹
        GunSet_AimbotShootFlag = 0;
        ShortTime_GunSet_AimbotShoot_Count = 0;
        LongTime_Accuracy_Check_Result = 0;
    }

    // 输出数据平滑
    if ((GimbalSet_GetFromVisual.PredictedTime - Last_Predicted_Tick > 200) || (Visual_Mode_Set_Last != Visual_Mode_Set))
    {
        // 判断本次模式与上次模式是否相同, 不相同或相隔时间太长时重置滤波器
        Smooth_SetData_Restart(&Smooth_VisualPitchSet, GimbalSet_Atti_Temp.Pitch);
        Smooth_SetData_Restart(&Smooth_VisualYawSet, GimbalSet_Atti_Temp.Yaw);
    }
    else
    {
        // 否则正常刷新平滑滤波器
        Smooth_SetDataABS(&Smooth_VisualPitchSet, GimbalSet_Atti_Temp.Pitch);
        Smooth_SetDataABS(&Smooth_VisualYawSet, GimbalSet_Atti_Temp.Yaw);
    }
    Smooth_GetDataABS(&GimbalSet_Atti_Temp.Pitch, &Smooth_VisualPitchSet);
    Smooth_GetDataABS(&GimbalSet_Atti_Temp.Yaw, &Smooth_VisualYawSet);
    // 记录新的相关数据
    Last_Predicted_Tick = GimbalSet_GetFromVisual.PredictedTime;
    Visual_Mode_Set_Last = Visual_Mode_Set;

    Gimbal_SetData_Out->GimbalSet_Atti.Pitch = GimbalSet_Atti_Temp.Pitch;
    Gimbal_SetData_Out->GimbalSet_Atti.Yaw = GimbalSet_Atti_Temp.Yaw;
    Gimbal_SetData_Out->GimbalSet_Speed.Pitch = GimbalSet_GetFromVisual.GimbalSet_Speed.Pitch;
    Gimbal_SetData_Out->GimbalSet_Speed.Yaw = GimbalSet_GetFromVisual.GimbalSet_Speed.Yaw;
    Gimbal_SetData_Out->PredictedTime = TickNow;
    Gimbal_SetData_Out->State = RT_EOK;
}

// 二代自瞄启动程序
int AimbotV2_Init(void)
{
    //默认俯仰角限幅+10~-10度
    Gimbal_Pitch_UpLim = 10;
    Gimbal_Pitch_DownLim = -10;
    Muzzle_V_REM = 14.0f; //默认计算弹速14.6m/s

    // 数据平滑模块, 用于平滑视觉给出的设定值
    Smooth_Init(&Smooth_VisualPitchSet, 0, 30);
    Smooth_Init(&Smooth_VisualYawSet, 0, 30);
    Smooth_SetDataFix(&Smooth_VisualYawSet, 360, 0, 1);

    // 初始化视觉通信相关
    Visual_Com_Init();

    return RT_EOK;
}

/**
 * @brief 在电控端向视觉开放开火和云台控制的权限
 * @author fwlh
 * @param  Enable           输入真时开放权限, 反之关闭相关权限
 */
void Aimbot_Enable_Visual_CtrlFire(bool Enable)
{
    if (Enable)
    {
        Exit_AimbotFlag = 0;       // 正常自瞄
        FireCtrl_AimbotLim_Set(1); // 允许视觉自动发弹
#ifdef CORE_USING_INFANTRY
        FireCtrl_VisualFineFire_EN(0); // 关闭精细发弹控制
#endif
    }
    else
    {
        Exit_AimbotFlag = 1;       // 强行不自瞄
        FireCtrl_AimbotLim_Set(0); // 不允许视觉自动发弹
#ifdef CORE_USING_INFANTRY
        // 关闭精细发弹控制
        FireCtrl_VisualFineFire_EN(0);
#endif
    }
}
