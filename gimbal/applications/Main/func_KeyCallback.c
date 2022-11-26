#include <func_KeyCallback.h>
#include <rtthread.h>
#include "func_Key_Record.h"
#include <drv_GimbalPublic.h>
#include <func_ModCTR.h>
#include "func_gun.h"
#include "drv_GunSettings.h"
#include "drv_UI_Flags.h"
#include "mod_aimbot_V2.h"
#include "drv_gundata.h"
#include "func_ModCTR.h"
#include "func_MonitorCfg.h"
#include "func_GimbalSet.h"
#ifndef CORE_USING_HERO
#include "drv_magazine.h"
#endif

// 记录当前菜单是否处于主菜单，此变量由ModKeyCTRL文件中的相关程序进行修改和刷新
// 用于在一维按键回调函数中进行查询，以实现使用二维菜单时不触发一维按键的功能的效果
char IS_MainMenu = 1; // 在主菜单时为1 初始位置一定在主菜单，故初始化为1
int current_menu = 0; // 当前所处的菜单

// 用于记录是否在主菜单的标志位修改函数
void MainMenu_REC_Entry_Fun(int TrigSource)
{
    IS_MainMenu = 1;
    current_menu = MainMenu;
}
void MainMenu_REC_Quit_Fun(int TrigSource)
{
    IS_MainMenu = 0;
}

/**
 * @brief：自瞄模式-按下回调函数
 * @param [in]   无
 * @return：		无
 * @author：zzj
 */
void Aimbot_PressCallback(void)
{
    // 重启平滑使得当前视觉设定值与电控设定值相等
    Smooth_Restart_VisualSet(Read_Real_Set(Pitch_Set), Read_Real_Set(Yaw_Set));
    Smooth_Restart_RobocontrolSet(Read_Real_Set(Pitch_Set), Read_Real_Set(Yaw_Set));
    Aimbot_Enable_Visual_CtrlFire(true); // 给予视觉控制机器人的权限
    Aimbot_FreshMouseClick(1);           // 向视觉发送鼠标按下的信息
}

/**
 * @brief：自瞄模式-弹起回调函数
 * @param [in]   无
 * @return：		无
 * @author：zzj
 */
void Aimbot_LoosenCallback(void)
{
    // 重启平滑使得当前视觉设定值与电控设定值相等
    Smooth_Restart_VisualSet(Read_Real_Set(Pitch_Set), Read_Real_Set(Yaw_Set));
    Smooth_Restart_RobocontrolSet(Read_Real_Set(Pitch_Set), Read_Real_Set(Yaw_Set));
    Aimbot_Enable_Visual_CtrlFire(false); // 关闭视觉控制机器人的权限
    Aimbot_FreshMouseClick(0);            // 向视觉发送鼠标未按下的信息
}

/**
 * @brief 探头模式-按下回调函数
 * @author fwlh
 */
void Probe_PressCallback(void)
{
    if (IS_MainMenu)
        Enter_Probe_Mode(1);
}

/**
 * @brief 探头模式-弹起回调函数
 * @author fwlh
 */
void Probe_LoosenCallback(void)
{
    if (IS_MainMenu)
        Enter_Probe_Mode(0);
}

/**
 * @brief 放开超级电容电量限制-按下回调函数
 * @author fwlh
 */
void SCAP_Reserve_PressCallback(void)
{
    if (IS_MainMenu)
        power_restrictions_open_flag ^= 1;
}

/**
 * @brief：主菜单回调：按住CTRL则修改枪口热量限制为无限，不按CTRL则修改发弹模式为三连发
 * @param [in]   无
 * @return：		无
 * @author：ych-zzj
 */
void MouseZ_P_Callback(int TrigSource)
{
    // 判断CTRL按键的状态
    if (Key_GetState(KeyEVT_CTRL))
    { // CTRL按下，需要解除热量限制
        GunSet_OverHeat_PermitFlag = 1;
        heatlimit_state = 1;
    }
#ifdef CORE_USING_HERO
    else
    {
        // 英雄没有三连发
    }
#else
    else
    {
        // 步兵，设置三连发模式
        Gun_mode_set(GUN_FAST);
    }
#endif
}
/**
 * @brief：主菜单回调：按住CTRL则修改枪口热量限制为限制，不按CTRL则修改发弹模式为自动模式
 * @param [in]   无
 * @return：		无
 * @author：ych-zzj
 */
void MouseZ_N_Callback(int TrigSource)
{
    // 判断CTRL按键的状态
    if (Key_GetState(KeyEVT_CTRL))
    { // CTRL按下，需要恢复热量限制
        GunSet_OverHeat_PermitFlag = 0;
        heatlimit_state = 0;
    }
#ifdef CORE_USING_HERO
    else
    {
        // 英雄没有三连发
    }
#else
    else
    {
        // 步兵，恢复单发模式
        Gun_mode_set(GUN_SLOW);
    }
#endif
}

/**
 * @brief：底盘运动模式-二维按键回调函数
 * @author：ych
 */
void MotionModeEntry_Callback(int TrigSource)
{
    if (TrigSource == CHASSISMODE_KEY_ENTRY)
        current_menu = ChassisModeMenu;
}
/**
 * @brief：底盘运动模式-二维按键回调函数
 * @author：ych
 */
void MotionModeSet_Callback(int TrigSource)
{
#if defined CORE_USING_HERO
    // 英雄在吊射模式下不能改变底盘模式
    if (Read_Dangling_Mode())
        return;
#endif
    switch (TrigSource)
    {
    case CHASSISMODE_KEY_MODE_NO_FOLLOW:
        motion_mode = NO_FOLLOW; //不跟随
        break;
    case CHASSISMODE_KEY_MODE_FOLLOW:
        motion_mode = FOLLOW_GIMBAL; //跟随
        break;
    case CHASSISMODE_KEY_MODE_SMALL_GYRO:
        motion_mode = SLOW_GYRO; //慢速陀螺
        break;
    case CHASSISMODE_KEY_MODE_FAST_GYRO:
        motion_mode = FAST_GYRO; //快速陀螺
        break;
    case CHASSISMODE_KEY_MODE_MOVE_BACK:
        Enter_MoveBack_Mode();
        break;
    default:
        break;
    }
}
/**
 * @brief：底盘运动模式-二维按键回调函数
 * @author：ych
 */
void MotionModeQuit_Callback(int TrigSource)
{
}

/**
 * @brief：自瞄模式设置-二维按键回调函数
 * @author：ych
 */
void AimMode_Entry_Callback(int TrigSource)
{ // 进入自瞄模式选择菜单
    current_menu = AimModeMenu;
}
int SetMaxSpeed = 15;
/**
 * @brief：自瞄模式设置-二维按键回调函数
 * @author：ych
 */
void AimMode_Set_Callback(int TrigSource)
{ // 按照按键情况修改自瞄模式
    switch (TrigSource)
    {
    case AIMMODE_SET_AIMBOT:
        // 常规自瞄模式
        // 修改标志 01：在这里添加自瞄模式切换相关函数
        ui_aimbot_mode = AIMBOT_MODE;
        Refresh_VisualMode(VISUAL_MODE_AIMBOT_V2);
        // 开启自动发弹检查
        FireCtrl_AimbotLim_Set(1);
#ifdef CORE_USING_INFANTRY
        // 关闭精细发弹控制
        FireCtrl_VisualFineFire_EN(0);
#endif
#ifdef CORE_USING_INFANTRY
        if (Read_Speed_Lim() > GUN_SPEED_REAL_30) // 设定弹速为 30m/s
            Gun_SpeedSet(GUN_SPEED_SET_30);
        else if (Read_Speed_Lim() > GUN_SPEED_REAL_18) // 设定弹速为 18m/s
            Gun_SpeedSet(GUN_SPEED_SET_18);
        else
            Gun_SpeedSet(GUN_SPEED_SET_15);
#elif defined CORE_USING_HERO
        Enter_Dangling_Mode(0);                   // 关闭吊射模式
        if (Read_Speed_Lim() > GUN_SPEED_REAL_16) // 设定弹速为 15m/s
            Gun_SpeedSet(GUN_SPEED_SET_16);
        else
            Gun_SpeedSet(GUN_SPEED_SET_10);
#endif
        break;
#if defined CORE_USING_HERO
    case AIMMODE_SET_ROTATING_OUTPOST:
        ui_aimbot_mode = ROTATING_OUTPOST;
        Refresh_VisualMode(VISUAL_MODE_AIMBUFF_ROTATING_OUTPOST);
        Enter_Dangling_Mode(0); // 关闭吊射模式
        // 开启自动发弹检查
        FireCtrl_AimbotLim_Set(1);
        if (Read_Speed_Lim() > GUN_SPEED_REAL_16) // 设定弹速为 16m/s
            Gun_SpeedSet(GUN_SPEED_SET_16);
        else
            Gun_SpeedSet(GUN_SPEED_SET_10);
        break;
    case AIMMODE_SET_STATIC_OUTPOST:
        ui_aimbot_mode = STATIC_OUTPOST;
        Refresh_VisualMode(VISUAL_MODE_AIMBUFF_STATIC_OUTPOST);
        Enter_Dangling_Mode(0); // 关闭吊射模式
        // 开启自动发弹检查
        FireCtrl_AimbotLim_Set(1);
        if (Read_Speed_Lim() > GUN_SPEED_REAL_16) // 设定弹速为 16m/s
            Gun_SpeedSet(GUN_SPEED_SET_16);
        else
            Gun_SpeedSet(GUN_SPEED_SET_10);
        break;
    case AIMMODE_SET_OUTPOST_F:
        ui_aimbot_mode = OUTPOST_MODE_F;
        Refresh_VisualMode(VISUAL_MODE_AIMBUFF_OUTPOST_F);
        Enter_Dangling_Mode(0); // 关闭吊射模式
        // 开启自动发弹检查
        FireCtrl_AimbotLim_Set(1);
        if (Read_Speed_Lim() > GUN_SPEED_REAL_16) // 设定弹速为 16m/s
            Gun_SpeedSet(GUN_SPEED_SET_16);
        else
            Gun_SpeedSet(GUN_SPEED_SET_10);
        break;
    case AIMMODE_SET_DANGLING_MODE:
        ui_aimbot_mode = DANGLING_MODE;
        Dangling_RecNow_MotionMode();                           // 进入吊射模式的瞬间记录一下当前的底盘模式, 从吊射模式恢复时自动恢复
        MotionModeSet_Callback(CHASSISMODE_KEY_MODE_NO_FOLLOW); // 由于吊射模式采用编码器闭环, 所以不能跟随
        Enter_Dangling_Mode(1);                                 // 进入吊射模式
        Smooth_Restart_RobocontrolSet(Read_Real_Set(Pitch_Set), Read_Real_Set(Yaw_Set));
        // 关闭自动发弹检查
        FireCtrl_AimbotLim_Set(0);
        if (Read_Speed_Lim() > GUN_SPEED_REAL_16) // 设定弹速为 15m/s
            Gun_SpeedSet(GUN_SPEED_SET_16);
        else
            Gun_SpeedSet(GUN_SPEED_SET_10);
        break;
#elif defined CORE_USING_INFANTRY /* CORE_USING_HERO */
    case AIMMODE_SET_AIMBUFF_CONST_SPEED:
        // 小能量机关
        // 修改标志 01：在这里添加自瞄模式切换相关函数
        ui_aimbot_mode = AIMBUFF_CONST_MODE;
        Refresh_VisualMode(VISUAL_MODE_AIMBUFF_CONST_SPEED);
        // 关闭自动发弹检查
        FireCtrl_AimbotLim_Set(0);
        // 开启精细发弹控制
        FireCtrl_VisualFineFire_EN(1);
        Visual_FineFire_FlagsRenew(VisualFlag_RuneFire);
        // 切换低射频模式
        Gun_mode_set(GUN_SLOW);
        // 能量机关不跟随
        MotionModeSet_Callback(CHASSISMODE_KEY_MODE_NO_FOLLOW);
        if (Read_Speed_Lim() > GUN_SPEED_REAL_30) // 设定弹速为 30m/s
            Gun_SpeedSet(GUN_SPEED_SET_30);
        else if (Read_Speed_Lim() > GUN_SPEED_REAL_18)
            Gun_SpeedSet(GUN_SPEED_SET_18);
        else if (Read_Speed_Lim() > GUN_SPEED_REAL_15)
            Gun_SpeedSet(GUN_SPEED_SET_15);
        break;
    case AIMMODE_SET_AIMBUFF_VARY_SPEED:
        // 大能量机关
        // 修改标志 01：在这里添加自瞄模式切换相关函数
        ui_aimbot_mode = AIMBUFF_VAR_MODE;
        Refresh_VisualMode(VISUAL_MODE_AIMBUFF_VARY_SPEED);
        // 切换低射频模式
        Gun_mode_set(GUN_SLOW);
        // 关闭自动发弹检查
        FireCtrl_AimbotLim_Set(0);
        // 开启精细发弹控制
        FireCtrl_VisualFineFire_EN(1);
        Visual_FineFire_FlagsRenew(VisualFlag_RuneFire);
        // 切换低射频模式
        Gun_mode_set(GUN_SLOW);
        // 能量机关不跟随
        MotionModeSet_Callback(CHASSISMODE_KEY_MODE_NO_FOLLOW);
        if (Read_Speed_Lim() > GUN_SPEED_REAL_30) // 设定弹速为 30m/s
            Gun_SpeedSet(GUN_SPEED_SET_30);
        else if (Read_Speed_Lim() > GUN_SPEED_REAL_18)
            Gun_SpeedSet(GUN_SPEED_SET_18);
        else if (Read_Speed_Lim() > GUN_SPEED_REAL_15)
            Gun_SpeedSet(GUN_SPEED_SET_15);
        break;
#endif
    default:
        break;
    }
}
/**
 * @brief：自瞄模式设置-二维按键回调函数
 * @author：ych
 */
void AimMode_Quit_Callback(int TrigSource)
{
}

/**
 * @brief 其他设置-二维按键回调函数
 * @author：ych
 */
void MiscEntry_Callback(int TrigSource)
{
    current_menu = MiscMenu;
}
/**
 * @brief 其他设置-二维按键回调函数
 * @author：ych
 */
void MiscSet_Callback(int TrigSource)
{
    switch (TrigSource)
    {
    case MISC_KEY_UI_RST:
        ui_reset ^= 1;
        break;
#ifndef CORE_USING_HERO
    case MISC_KEY_HATCH_OPEN:
        // 自动开弹舱
        magazine_state = 1;
        Magazine_servo_set(SERVO_OPEN);
        FillMode_EN = 1;
        break;
    case MISC_KEY_HATCH_CLSE:
        // 自动关弹舱
        magazine_state = 0;
        Magazine_servo_set(SERVO_CLOSE);
        FillMode_EN = 0;
        break;
#endif
    default:
        break;
    }
}
/**
 * @brief 其他设置-二维按键回调函数
 * @author：ych
 */
void MiscQuit_Callback(int TrigSource)
{
}

/**
 * @brief 单片机复位选择菜单的进入函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
void ResetEntry_Callback(int TrigSource)
{
    current_menu = RobotResetMenu;
}

/**
 * @brief 单片机复位选择菜单的设置函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
void Reset_Callback(int TrigSource)
{
    // 如果此时 Ctrl 键没有按下就直接退出
    if (!Key_GetState(KeyEVT_CTRL))
        return;
    switch (TrigSource)
    {
    case MISC_KEY_GIMBAL_RESET:
        Robot_Reset_Gimbal();
        break;
    case MISC_KEY_CHASSIS_RESET:
        ResetCmd_Write(1);
        chassis_data_send(); // 该命令即刻生效, 所以需要调用一次发送
        break;
    case MISC_KEY_TOTAL_RESET:
        ResetCmd_Write(1);
        chassis_data_send(); // 该命令即刻生效, 所以需要调用一次发送
        Robot_Reset_Gimbal();
        break;
    default:
        break;
    }
}

/**
 * @brief 单片机复位选择菜单的退出函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
void ResetExit_Callback(int TrigSource)
{
}

/**
 * @brief 超级电容充电开关选择菜单的进入函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
void SCAP_Ctrl_Entry_Callback(int TrigSource)
{
    current_menu = SCAPCtrlMenu;
}

/**
 * @brief 超级电容充电开关选择菜单的设置函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
void SCAP_Ctrl_Callback(int TrigSource)
{
    switch (TrigSource)
    {
    case MISC_KEY_CAP_OPEN:
        capacity_close = 0;
        break;
    case MISC_KEY_CAP_CLSE:
        capacity_close = 1;
        break;
    default:
        break;
    }
}

/**
 * @brief 超级电容充电开关选择菜单的退出函数-二维按键回调函数
 * @author fwlh
 * @param  TrigSource       按下的按键
 */
void SCAP_Ctrl_Exit_Callback(int TrigSource)
{
}

/**
 * @brief：弹速设置-二维按键回调函数
 * @author：zzj
 */
void GunSpeedSet_Entry_Callback(int TrigSource)
{ // 进入弹速设置选择菜单
    current_menu = GunSpeedMenu;
}

/**
 * @brief：弹速设置-二维按键回调函数
 * @author：zzj
 */
void GunSpeedSet_Set_Callback(int TrigSource)
{
    switch (TrigSource)
    {
#ifdef CORE_USING_INFANTRY
    case GUNSPEED_SET_15:
        Gun_SpeedSet(GUN_SPEED_SET_15);
        break;
    case GUNSPEED_SET_18:
        if (Read_Speed_Lim() > GUN_SPEED_REAL_18)
            Gun_SpeedSet(GUN_SPEED_SET_18);
        break;
    case GUNSPEED_SET_30:
        if (Read_Speed_Lim() > GUN_SPEED_REAL_30)
            Gun_SpeedSet(GUN_SPEED_SET_30);
        break;
#elif defined CORE_USING_HERO
    case GUNSPEED_SET_10:
        Gun_SpeedSet(GUN_SPEED_SET_10);
        break;
    case GUNSPEED_SET_16:
        if (Read_Speed_Lim() > GUN_SPEED_REAL_16)
            Gun_SpeedSet(GUN_SPEED_SET_16);
        break;
#endif
    default:
        break;
    }
}

/**
 * @brief：弹速设置-二维按键回调函数
 * @author：zzj
 */
void GunSpeedSet_Quit_Callback(int TrigSource)
{
}

void PerformanceEnter_Callback(int Trig_Key)
{
    current_menu = PerformanceMenu;
}

void Performance_Callback(int Trig_Key)
{
    switch (Trig_Key)
    {
    case PERFORMANCE_KEY_LEVEL1:
        Local_Robot_Level_Fresh(Level1);
        break;
    case PERFORMANCE_KEY_LEVEL2:
        Local_Robot_Level_Fresh(Level2);
        break;
    case PERFORMANCE_KEY_LEVEL3:
        Local_Robot_Level_Fresh(Level3);
        break;
    default:
        break;
    }
    // 更新底盘功率上限参数
    LocalHeat_Data_Fresh_Limit(&GimbalSetPower);
}

void PerformanceQuit_Callback(int Trig_Key)
{
}

void PerformanceChooseEnter_Callback(int Trig_Key)
{
    current_menu = PerformanceChooseMenu;
}

void PerformanceChoose_Callback(int Trig_Key)
{
    switch (Trig_Key)
    {
    case PERFORMANCE_TYPE_KEY_OUTBREAK:
        Local_Robot_Type_Fresh(Outbreak_Priority);
        break;
#ifndef CORE_USING_HERO
    case PERFORMANCE_TYPE_KEY_COOLING:
        Local_Robot_Type_Fresh(Cooling_Priority);
        break;
#endif
    case PERFORMANCE_TYPE_KEY_BULLETSPEED:
        Local_Robot_Type_Fresh(BulletSpeed_Priority);
        break;
    case PERFORMANCE_TYPE_KEY_POWER:
        Local_Robot_Type_Fresh(Power_Priority);

        break;
    case PERFORMANCE_TYPE_KEY_BLOOD:
        Local_Robot_Type_Fresh(Blood_Priority);
        break;
    case PERFORMANCE_TYPE_KEY_FORCED_OFFLINE:
        force_refsystem_offline = 1;
        break;
    case PERFORMANCE_TYPE_KEY_FORCED_ONLINE:
        force_refsystem_offline = 0;
        break;
    default:
        break;
    }
}

void PerformanceChooseQuit_Callback(int Trig_Key)
{
}
