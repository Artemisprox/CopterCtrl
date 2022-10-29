#include "func_GimbalSet.h"
#include "mod_aimbot_V2.h"
#include "drv_motor.h"
#include "func_GimbalFB.h"
#include <app_robocontrol.h>
#include "drv_Queue.h"
#include "drv_remote.h"
#include "drv_utils.h"

#define SETPLANNING_ROBOCONTROL_ACCLMAX_PITCH 1500
#define SETPLANNING_ROBOCONTROL_SPEEDMAX_PITCH 500
#define SETPLANNING_ROBOCONTROL_ACCLMAX_YAW 3000
#define SETPLANNING_ROBOCONTROL_SPEEDMAX_YAW 600

#define SETPLANNING_AIMBOT_ACCLMAX_PITCH 1300
#define SETPLANNING_AIMBOT_SPEEDMAX_PITCH 500
#define SETPLANNING_AIMBOT_ACCLMAX_YAW 2400
#define SETPLANNING_AIMBOT_SPEEDMAX_YAW 500

pid_t PitchANG_ENCD_Settings, PitchANG_IMU_Settings;
pid_t YawANG_ENCD_Settings, YawANG_IMU_Settings;
pid_t PitchSPE_ENCD_Settings, PitchSPE_IMU_Settings;
pid_t YawSPE_ENCD_Settings, YawSPE_IMU_Settings;
#if defined CORE_USING_HERO
pid_t PitchSPE_Dangling_Settings, PitchANG_Dangling_Settings;
#endif

static SetPlanning_Str SetPitch, SetYaw;     // Pitch 和 Yaw 轴的设定值规划模块
static float CurrentSetPitch, CurrentSetYaw; // 当前准备设定的 Pitch 和 Yaw 轴的设定值(展开后至(-∞, +∞)后)

ExactSmth_CTRL_S Smooth_PitchAngleSet, Smooth_YawAngleSet;

// 记录当前云台姿态角限位
GimbalLiPItch_Type Gimbal_PitchLim_IMU, Gimbal_PitchLim_ENCD; // IMU反馈时限幅、编码器反馈时限幅

Gimbal_SetCal_Type Gimbal_SetData_Out;       // 定义一个用来查询二代自瞄状态的结构体
AttitudeData_Type RoboControl_GimbalSet_ADD; // 从遥控器端获得的云台设定值增量
AttitudeData_Type Visual_GimbalSet_ABS;      // 从视觉端端获得的云台设定值

int Exit_AimbotFlag = 1; //为1时强行退出自瞄
#if defined CORE_USING_HERO
rt_uint8_t DanglingMode_Flag = 0; // 标志当前是否处在吊射模式中
#endif                            /* CORE_USING_HERO */

static GimbalFBSelection_E FB_Selection_Last_Pitch = FB_IMU; // 上一次使用的数据源
static GimbalFBSelection_E FB_Selection_Last_Yaw = FB_IMU;   // 上一次使用的数据源
static GimbalFBSelection_E FB_Selection_Set_Pitch = FB_IMU;  // 当前预期数据源
static GimbalFBSelection_E FB_Selection_Set_Yaw = FB_IMU;    // 当前预期数据源

// 修改当前云台预期数据源
void Gimbal_FBS_Set_Pitch(GimbalFBSelection_E Set)
{
    FB_Selection_Set_Pitch = Set; // 直接设定 Feedback Selection
}
void Gimbal_FBS_Set_Yaw(GimbalFBSelection_E Set)
{
    FB_Selection_Set_Yaw = Set; // 直接设定 Feedback Selection
}

#if defined CORE_USING_HERO
// 用于退出或进入吊射模式
void Enter_Dangling_Mode(rt_uint8_t Enter)
{
    DanglingMode_Flag = Enter;
}

// 读取当前是否处于吊射模式中
rt_uint8_t Read_Dangling_Mode(void)
{
    return DanglingMode_Flag;
}
#endif /* CORE_USING_HERO */

/**
 * @brief 云台姿态限幅计算和更新函数
 * @return 无
 * @author ych
 */
static void Gimbal_Atti_Lim_Cal(void)
{ // 计算当前云台设定值的限幅(IMU) 编码器限幅数据在初始化时已固定
    Gimbal_PitchLim_IMU.DownLim = -GimbalFB.DeltaAtti.Pitch + PITCH_MIN_ANGLE;
    Gimbal_PitchLim_IMU.UpLim = Gimbal_PitchLim_IMU.DownLim + PITCH_MAX_ANGLE - PITCH_MIN_ANGLE;
}

// 刷新：云台闭环数据源检查（限位附近强制使用编码器，其余优先使用设置中给定的数据源
static int GimbalPitch_LimState = 0; // 0表示没有触发限位，1表示触发UpLim，-1表示触发DownLim
void Gimbal_FB_Selection_Fresh(void)
{
#if defined CORE_USING_HERO
    // 吊射模式下不需要重新判断
    if (DanglingMode_Flag)
        return;
#endif /* CORE_USING_HERO */
    // 刷新当前限位状态数据
    switch (GimbalPitch_LimState)
    {
    default:
    case 0:
        // 当前没有处在限位位置
        // 计算当前编码器与限位范围的关系
        if (GimbalFB.ENCD_ATTI.Pitch > Gimbal_PitchLim_ENCD.UpLim)
            GimbalPitch_LimState = 1;
        else if (GimbalFB.ENCD_ATTI.Pitch < Gimbal_PitchLim_ENCD.DownLim)
            GimbalPitch_LimState = -1;
        break;
    case 1:
        // 处于顶部限位区域
        if (GimbalFB.ENCD_ATTI.Pitch < Gimbal_PitchLim_ENCD.DownLim)
            GimbalPitch_LimState = -1;
        else if (GimbalFB.ENCD_ATTI.Pitch < Gimbal_PitchLim_ENCD.UpLim - GIMBAL_LIM_LEN)
            // 如果超出滞回比较区域，则恢复未触发限位状态
            GimbalPitch_LimState = 0;
        break;
    case -1:
        // 处于底部限位区域
        if (GimbalFB.ENCD_ATTI.Pitch > Gimbal_PitchLim_ENCD.UpLim)
            GimbalPitch_LimState = 1;
        else if (GimbalFB.ENCD_ATTI.Pitch > Gimbal_PitchLim_ENCD.DownLim + GIMBAL_LIM_LEN)
            // 如果超出滞回比较区域，则恢复未触发限位状态
            GimbalPitch_LimState = 0;
        break;
    }

    if (GimbalPitch_LimState == 0)
        // 当前没有限位，使用预期数据源
        GimbalPitch_FB_Select_Set(FB_Selection_Set_Pitch);
    else
        // 处于限位区域，强制选用编码器闭环
        GimbalPitch_FB_Select_Set(FB_ENCD);
    GimbalYaw_FB_Select_Set(FB_Selection_Set_Yaw);
}

volatile int PitchPID_Num_JSCOPE = 0;
volatile int YawPID_Num_JSCOPE = 0;

static GimbalFBSelection_E PitchFBS_Rec = FB_NONE;
static GimbalFBSelection_E YawFBS_Rec = FB_NONE;
// 按需修改PID闭环参数
static void PID_ConfirmSettings(GimbalFBSelection_E PitchFBS, GimbalFBSelection_E YawFBS)
{
#if defined CORE_USING_HERO
    if (DanglingMode_Flag)
    {
        Pitch.spe.i_limit = PitchSPE_Dangling_Settings.i_limit;
        Pitch.spe.kp = PitchSPE_Dangling_Settings.kp;
        Pitch.spe.ki = PitchSPE_Dangling_Settings.ki;
        Pitch.spe.kd = PitchSPE_Dangling_Settings.kd;
        Pitch.spe.out_limit_down = PitchSPE_Dangling_Settings.out_limit_down;
        Pitch.spe.out_limit_up = PitchSPE_Dangling_Settings.out_limit_up;

        Pitch.ang.i_limit = PitchANG_Dangling_Settings.i_limit;
        Pitch.ang.kp = PitchANG_Dangling_Settings.kp;
        Pitch.ang.ki = PitchANG_Dangling_Settings.ki;
        Pitch.ang.kd = PitchANG_Dangling_Settings.kd;
        Pitch.ang.out_limit_down = PitchANG_Dangling_Settings.out_limit_down;
        Pitch.ang.out_limit_up = PitchANG_Dangling_Settings.out_limit_up;
        PitchPID_Num_JSCOPE = 0;
    }
    else if (PitchFBS_Rec != PitchFBS)
#else
    if (PitchFBS_Rec != PitchFBS)
#endif
    {
        if (PitchFBS == FB_IMU)
        {
            Pitch.spe.i_limit = PitchSPE_IMU_Settings.i_limit;
            Pitch.spe.kp = PitchSPE_IMU_Settings.kp;
            Pitch.spe.ki = PitchSPE_IMU_Settings.ki;
            Pitch.spe.kd = PitchSPE_IMU_Settings.kd;
            Pitch.spe.out_limit_down = PitchSPE_IMU_Settings.out_limit_down;
            Pitch.spe.out_limit_up = PitchSPE_IMU_Settings.out_limit_up;

            Pitch.ang.i_limit = PitchANG_IMU_Settings.i_limit;
            Pitch.ang.kp = PitchANG_IMU_Settings.kp;
            Pitch.ang.ki = PitchANG_IMU_Settings.ki;
            Pitch.ang.kd = PitchANG_IMU_Settings.kd;
            Pitch.ang.out_limit_down = PitchANG_IMU_Settings.out_limit_down;
            Pitch.ang.out_limit_up = PitchANG_IMU_Settings.out_limit_up;
            PitchPID_Num_JSCOPE = 0;
        }
        else
        { // 默认使用编码器闭环参数
            Pitch.spe.i_limit = PitchSPE_ENCD_Settings.i_limit;
            Pitch.spe.kp = PitchSPE_ENCD_Settings.kp;
            Pitch.spe.ki = PitchSPE_ENCD_Settings.ki;
            Pitch.spe.kd = PitchSPE_ENCD_Settings.kd;
            Pitch.spe.out_limit_down = PitchSPE_ENCD_Settings.out_limit_down;
            Pitch.spe.out_limit_up = PitchSPE_ENCD_Settings.out_limit_up;

            Pitch.ang.i_limit = PitchANG_ENCD_Settings.i_limit;
            Pitch.ang.kp = PitchANG_ENCD_Settings.kp;
            Pitch.ang.ki = PitchANG_ENCD_Settings.ki;
            Pitch.ang.kd = PitchANG_ENCD_Settings.kd;
            Pitch.ang.out_limit_down = PitchANG_ENCD_Settings.out_limit_down;
            Pitch.ang.out_limit_up = PitchANG_ENCD_Settings.out_limit_up;
            PitchPID_Num_JSCOPE = 1;
        }
        PitchFBS_Rec = PitchFBS;
    }
    if (YawFBS_Rec != YawFBS)
    {
        if (YawFBS == FB_IMU)
        {
            Yaw.spe.i_limit = YawSPE_IMU_Settings.i_limit;
            Yaw.spe.kp = YawSPE_IMU_Settings.kp;
            Yaw.spe.ki = YawSPE_IMU_Settings.ki;
            Yaw.spe.kd = YawSPE_IMU_Settings.kd;
            Yaw.spe.out_limit_down = YawSPE_IMU_Settings.out_limit_down;
            Yaw.spe.out_limit_up = YawSPE_IMU_Settings.out_limit_up;

            Yaw.ang.i_limit = YawANG_IMU_Settings.i_limit;
            Yaw.ang.kp = YawANG_IMU_Settings.kp;
            Yaw.ang.ki = YawANG_IMU_Settings.ki;
            Yaw.ang.kd = YawANG_IMU_Settings.kd;
            Yaw.ang.out_limit_down = YawANG_IMU_Settings.out_limit_down;
            Yaw.ang.out_limit_up = YawANG_IMU_Settings.out_limit_up;
            YawPID_Num_JSCOPE = 0;
        }
        else
        { // 默认使用编码器闭环参数
            Yaw.spe.i_limit = YawSPE_ENCD_Settings.i_limit;
            Yaw.spe.kp = YawSPE_ENCD_Settings.kp;
            Yaw.spe.ki = YawSPE_ENCD_Settings.ki;
            Yaw.spe.kd = YawSPE_ENCD_Settings.kd;
            Yaw.spe.out_limit_down = YawSPE_ENCD_Settings.out_limit_down;
            Yaw.spe.out_limit_up = YawSPE_ENCD_Settings.out_limit_up;

            Yaw.ang.i_limit = YawANG_ENCD_Settings.i_limit;
            Yaw.ang.kp = YawANG_ENCD_Settings.kp;
            Yaw.ang.ki = YawANG_ENCD_Settings.ki;
            Yaw.ang.kd = YawANG_ENCD_Settings.kd;
            Yaw.ang.out_limit_down = YawANG_ENCD_Settings.out_limit_down;
            Yaw.ang.out_limit_up = YawANG_ENCD_Settings.out_limit_up;
            YawPID_Num_JSCOPE = 1;
        }
        YawFBS_Rec = YawFBS;
    }
}

// 云台限幅计算和设定值强制切换处理
static GimbalFBSelection_E FB_Now_Pitch, FB_Now_Yaw; // 便于Jscope观察
static int LimRecover_Count = 0;
static void GimbalSet_Fix(GimbalCTRL_Set_Type *Gimbal_Setang, float *SetPlanning_DeltaPitch)
{
    // 设定值结构体传入时，其中存有当前期望的设定值。
    // 按照数据源分类讨论，检查是否发生过数据源切换
    FB_Now_Pitch = GimbalPitch_FB_Select_Get();
    if (FB_Selection_Last_Pitch != FB_Now_Pitch)
    {
        // 出现数据源切换
        if (FB_Selection_Last_Pitch == FB_ENCD)
        {
            // 从编码器切换到陀螺仪
            Gimbal_Setang->Pitch -= GimbalFB.DeltaAtti.Pitch;
            *SetPlanning_DeltaPitch = -GimbalFB.DeltaAtti.Pitch;

            if (Gimbal_Setang->Pitch > 90)
                Gimbal_Setang->Pitch = 90;
            else if (Gimbal_Setang->Pitch < -90)
                Gimbal_Setang->Pitch = -90;
        }
        else
            // 从陀螺仪切换到编码器, 无需处理
            *SetPlanning_DeltaPitch = 0.f;
        FB_Selection_Last_Pitch = FB_Now_Pitch;
    }
    else
        // 没有出现数据切换, 无需处理
        *SetPlanning_DeltaPitch = 0.f;

    FB_Now_Yaw = GimbalYaw_FB_Select_Get();
    if (FB_Selection_Last_Yaw != FB_Now_Yaw)
    {
        // 出现数据源切换
        if (FB_Selection_Last_Yaw == FB_ENCD)
            // 从编码器切换到陀螺仪
            Gimbal_Setang->Yaw -= GimbalFB.DeltaAtti.Yaw;
        // 从陀螺仪切换到编码器, 无需处理
        // 记录数据切换
        FB_Selection_Last_Yaw = FB_Now_Yaw;
    }
    else
        // 没有出现数据切换, 无需处理
        PID_ConfirmSettings(FB_Now_Pitch, FB_Now_Yaw);

    // 检查是否需要回到正常控制区间
    if (GimbalPitch_LimState != 0)
    {
        if (GimbalPitch_LimState == 1)
        {
            // 当前位于顶部限位
            if (GimbalFB.DeltaAtti.Pitch < -GIMBAL_LIM_LEN)
            {
                // 滞回比较
                if (LimRecover_Count < 200)
                    LimRecover_Count++;
                else if (LimRecover_Count > 0)
                    LimRecover_Count--;
            }
            if (LimRecover_Count > 100)
                if (Gimbal_Setang->Pitch > Gimbal_PitchLim_ENCD.UpLim - 2 * GIMBAL_LIM_LEN)
                    Gimbal_Setang->Pitch -= 3 / 1000.0f; // 3dps角速度回正
        }
        else
        {
            // 当前位于底部限位
            if (GimbalFB.DeltaAtti.Pitch > GIMBAL_LIM_LEN)
            {
                // 滞回比较
                if (LimRecover_Count < 200)
                    LimRecover_Count++;
                else if (LimRecover_Count > 0)
                    LimRecover_Count--;
            }
            if (LimRecover_Count > 100)
                if (Gimbal_Setang->Pitch < Gimbal_PitchLim_ENCD.DownLim + 2 * GIMBAL_LIM_LEN)
                    Gimbal_Setang->Pitch += 3 / 1000.0f; // 3dps角速度回正
        }
    }
    else
        LimRecover_Count = 0;

    // 按照当前采用的数据源进行设定值限幅
    if (FB_Now_Pitch == FB_ENCD)
    {
        // 正在使用编码器闭环，使用编码器数据进行设定值限幅
        if (Gimbal_Setang->Pitch > Gimbal_PitchLim_ENCD.UpLim)
            Gimbal_Setang->Pitch = Gimbal_PitchLim_ENCD.UpLim;
        else if (Gimbal_Setang->Pitch < Gimbal_PitchLim_ENCD.DownLim)
            Gimbal_Setang->Pitch = Gimbal_PitchLim_ENCD.DownLim;
    }
    else
    {
        // 使用陀螺仪闭环，设定值限幅：
        if (Gimbal_Setang->Pitch > Gimbal_PitchLim_IMU.UpLim + GIMBAL_LIM_LEN)
            Gimbal_Setang->Pitch = Gimbal_PitchLim_IMU.UpLim + GIMBAL_LIM_LEN;
        else if (Gimbal_Setang->Pitch < Gimbal_PitchLim_IMU.DownLim - GIMBAL_LIM_LEN)
            Gimbal_Setang->Pitch = Gimbal_PitchLim_IMU.DownLim - GIMBAL_LIM_LEN;
    }
}

static GimbalCTRL_Set_Type SetAng_BeforePlanning;
/**
 * @brief 云台设定值获取主函数
 * @param [GimbalCTRL_Set_Type*] Gimbal_Setang：云台设定值结构体 调用函数前为上一次的设定值，调用后为这一次的设定值
 * @return 无
 * @author ych
 */
void Gimbal_getset(GimbalCTRL_Set_Type *Gimbal_Setang)
{
    float SetPlanning_DeltaPitch;
    float TempSet;
#if defined CORE_USING_HERO
    /* 云台姿态限幅计算和更新函数 */
    if (DanglingMode_Flag)
    {
        // 吊射模式下直接选择编码器闭环
        GimbalPitch_FB_Select_Set(FB_IMU);
        GimbalYaw_FB_Select_Set(FB_IMU);
    }
    else
    {
        // 否则优先选用 IMU
        GimbalPitch_FB_Select_Set(FB_IMU);
        GimbalYaw_FB_Select_Set(FB_IMU);
    }
#endif /* CORE_USING_HERO */
    // 反馈基础数据计算
    Gimbal_PID_FB_Fresh();
    // IMU动态限幅范围计算
    Gimbal_Atti_Lim_Cal();
    // 检查是否需要切换数据源
    Gimbal_FB_Selection_Fresh();

    /* 限幅发送函数（发送给二代自瞄函数，作用于弹道计算） */
    Refresh_Gimbal_Lim(Gimbal_PitchLim_IMU.UpLim, Gimbal_PitchLim_IMU.DownLim);

    /* 运行二代自瞄设定值获取 并接收自瞄状态(内部已经使用平滑) */
    Aimbot_Get_GimbalSet(&Gimbal_SetData_Out);
    UTILS_NAN_ZERO_F(Gimbal_SetData_Out.GimbalSet_Atti.Pitch);
    UTILS_NAN_ZERO_F(Gimbal_SetData_Out.GimbalSet_Speed.Pitch);
    UTILS_NAN_ZERO_F(Gimbal_SetData_Out.GimbalSet_Atti.Yaw);
    UTILS_NAN_ZERO_F(Gimbal_SetData_Out.GimbalSet_Speed.Yaw);
    /* 运行遥控器设定值增量获取 */
    Smooth_GetDataADD(&RoboControl_GimbalSet_ADD.Pitch, &Smooth_PitchAngleSet);
    Smooth_GetDataADD(&RoboControl_GimbalSet_ADD.Yaw, &Smooth_YawAngleSet);
    UTILS_NAN_ZERO_F(RoboControl_GimbalSet_ADD.Pitch);
    UTILS_NAN_ZERO_F(RoboControl_GimbalSet_ADD.Yaw);

    /* 判断二代自瞄是否锁定目标 */
    // 获得展开后的 Yaw 轴真实角度
    SetYaw.Input.Now.pos = utils_angle_difference(GimbalFB.FB_This.Yaw, SetYaw.Input.Now.pos) + SetYaw.Input.Now.pos;
    if ((Gimbal_SetData_Out.State == RT_EOK) && (Exit_AimbotFlag == 0))
    {                                       // 有锁定目标
        Gimbal_Setang->Set_Source = Aimbot; // 修改控制源

        SetPitch.Settings.Accl_Max = SETPLANNING_AIMBOT_ACCLMAX_PITCH;
        SetPitch.Settings.Speed_Max = SETPLANNING_AIMBOT_SPEEDMAX_PITCH;
        SetYaw.Settings.Accl_Max = SETPLANNING_AIMBOT_ACCLMAX_YAW;
        SetYaw.Settings.Speed_Max = SETPLANNING_AIMBOT_SPEEDMAX_YAW;
        // 更新设定值
        TempSet = Gimbal_SetData_Out.GimbalSet_Atti.Pitch;
        utils_truncate_number(&TempSet, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE); // Pitch 轴设定值需要限幅
        CurrentSetPitch = TempSet;
        CurrentSetYaw = utils_angle_difference(Gimbal_SetData_Out.GimbalSet_Atti.Yaw, SetYaw.Input.Set.pos) + SetYaw.Input.Set.pos;
        SetPitch.Input.Set.spe = Gimbal_SetData_Out.GimbalSet_Speed.Pitch;
        SetYaw.Input.Set.spe = Gimbal_SetData_Out.GimbalSet_Speed.Yaw;
    }
    else
    { // 强行不自瞄，正常更新遥控器数据
        Gimbal_Setang->Set_Source = RoboControl;

        SetPitch.Settings.Accl_Max = SETPLANNING_ROBOCONTROL_ACCLMAX_PITCH;
        SetPitch.Settings.Speed_Max = SETPLANNING_ROBOCONTROL_SPEEDMAX_PITCH;
        SetYaw.Settings.Accl_Max = SETPLANNING_ROBOCONTROL_ACCLMAX_YAW;
        SetYaw.Settings.Speed_Max = SETPLANNING_ROBOCONTROL_SPEEDMAX_YAW;
        // 更新设定值
        TempSet = SetPitch.Input.Set.pos + RoboControl_GimbalSet_ADD.Pitch;
        utils_truncate_number(&TempSet, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE); // Pitch 轴设定值需要限幅
        CurrentSetPitch = TempSet;
        // Yaw 轴设定值跨圈处理
        TempSet = SetYaw.Input.Set.pos + RoboControl_GimbalSet_ADD.Yaw;
        if (SetYaw.Input.Now.pos - TempSet > 180.f)
            CurrentSetYaw = TempSet + 360.f;
        else if (SetYaw.Input.Now.pos - TempSet < -180.f)
            CurrentSetYaw = TempSet - 360.f;
        else
            CurrentSetYaw = TempSet;
        SetPitch.Input.Set.spe = 0;
        SetYaw.Input.Set.spe = 0;
    }
    SetAng_BeforePlanning.Pitch = CurrentSetPitch;
    SetAng_BeforePlanning.Yaw = CurrentSetYaw;

    /* 限位、数据源修正、过渡等 */
    // Pitch 轴设定值在数据源切换时需要修改
    GimbalSet_Fix(&SetAng_BeforePlanning, &SetPlanning_DeltaPitch);
    SetPlanning_SetOutput(&SetPitch, SetPitch.Output.pos + SetPlanning_DeltaPitch);

    SetYaw.Input.Set.pos = SetAng_BeforePlanning.Yaw;
    SetPitch.Input.Now.pos = GimbalFB.FB_This.Pitch;
    SetPitch.Input.Set.pos = SetAng_BeforePlanning.Pitch;

    /* 设定值规划 */
    SetPlanning_Cal(&SetYaw);
    SetPlanning_Cal(&SetPitch);
    /* 输出结果 */
    float temp = SetYaw.Output.pos + 180.f;
    utils_norm_angle(&temp);
    Gimbal_Setang->Yaw = temp - 180.f;
    Gimbal_Setang->Pitch = SetPitch.Output.pos;
}

// 用于外部读取当前的实际角度设定值
float Read_Real_Set(SetData_Type_Enum Data)
{
    switch (Data)
    {
    case Pitch_Set:
        return CurrentSetPitch;
    case Yaw_Set:
    {
        static float Temp = 0.f;
        Temp = CurrentSetYaw + 180.f;
        utils_norm_angle(&Temp);
        return (Temp - 180.f);
    }
    default:
        return 0.f;
    }
}

// 设定值获取初始化
void GimbalSet_Init(GimbalCTRL_Set_Type *SetSTR, float PitchSet, float YawSet)
{
    // 初始化设定值规划模块
    SetPlanSettings_Str Setting;
    Setting.dt = 1e-3f;
    Setting.POS_Error_Max = 3;
    Setting.Accl_Max = SETPLANNING_ROBOCONTROL_ACCLMAX_PITCH;
    Setting.Speed_Max = SETPLANNING_ROBOCONTROL_SPEEDMAX_PITCH;
    SetPlanning_Init(&SetPitch, &Setting);
    Setting.POS_Error_Max = 6;
    Setting.Accl_Max = SETPLANNING_ROBOCONTROL_ACCLMAX_YAW;
    Setting.Speed_Max = SETPLANNING_ROBOCONTROL_SPEEDMAX_YAW;
    SetPlanning_Init(&SetYaw, &Setting);
    // 数据平滑模块, 用于数据接力
    Smooth_Init(&Smooth_PitchAngleSet, 0, 15);
    Smooth_Init(&Smooth_YawAngleSet, 0, 15);

    GimbalFB_Init();                   // 初始化反馈数据计算结构体
    GimbalYaw_FB_Select_Set(FB_IMU);   // 默认使用IMU反馈
    GimbalPitch_FB_Select_Set(FB_IMU); // 默认使用IMU反馈

    SetSTR->Set_Source = Aimbot; //这里设定为上次获取数据的来源是自瞄，则上电后先调用遥控器控制时，平滑程序会正常进行开启时的过渡。
    SetSTR->Pitch = PitchSet;
    SetSTR->Yaw = YawSet;
    Gimbal_PitchLim_ENCD.DownLim = PITCH_MIN_ANGLE;
    Gimbal_PitchLim_ENCD.UpLim = PITCH_MAX_ANGLE;
#if (TEST_CLEAR_PID == 0)
    pid_init(&PitchANG_ENCD_Settings, PITANG_ENCOD_PID);
    pid_init(&PitchSPE_ENCD_Settings, PITSPE_ENCOD_PID);
    pid_init(&PitchANG_IMU_Settings, PITANG_PID);
    pid_init(&PitchSPE_IMU_Settings, PITSPE_PID);
    pid_init(&YawANG_ENCD_Settings, YAWANG_ENCOD_PID);
    pid_init(&YawSPE_ENCD_Settings, YAWSPE_ENCOD_PID);
    pid_init(&YawANG_IMU_Settings, YAWANG_PID);
    pid_init(&YawSPE_IMU_Settings, YAWSPE_PID);
#if defined CORE_USING_HERO
    pid_init(&PitchSPE_Dangling_Settings, PITSPE_DANGLING_PID);
    pid_init(&PitchANG_Dangling_Settings, PITANG_DANGLING_PID);
#endif
#else
    pid_init(&PitchANG_ENCD_Settings, PID_CLEAR);
    pid_init(&PitchSPE_ENCD_Settings, PID_CLEAR);
    pid_init(&PitchANG_IMU_Settings, PID_CLEAR);
    pid_init(&PitchSPE_IMU_Settings, PID_CLEAR);
    pid_init(&YawANG_ENCD_Settings, PID_CLEAR);
    pid_init(&YawSPE_ENCD_Settings, PID_CLEAR);
    pid_init(&YawANG_IMU_Settings, PID_CLEAR);
    pid_init(&YawSPE_IMU_Settings, PID_CLEAR);
#if defined CORE_USING_HERO
    pid_init(&PitchSPE_Dangling_Settings, PID_CLEAR);
    pid_init(&PitchANG_Dangling_Settings, PID_CLEAR);
#endif
#endif
}
