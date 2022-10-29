#include <app_robocontrol.h>
#include "drv_thread.h"
#include <func_ModCTR.h>
#include "drv_remote.h"
#include "mod_aimbot_V2.h"
#include "func_gun.h"
#include "drv_GimbalPublic.h"
#include "drv_Aimbot_Public.h"
#include "drv_buzzer.h"
#include "mod_Monitor.h"
#include "drv_GunSettings.h"
#include "func_GimbalSet.h"
#include "func_MonHandling.h"

char ComputerCTRL_EN = 0; // 是否为客户端模式

int gunspeed = 0; // 弹速设定值

#define COMPUTER 0 // 客户端模式
#define REMOTE 1   // 遥控器模式
#define GENERAL 2  // 通用处理

static void General_Key_Action_Process(switch_action_e *s1_action, switch_action_e *s2_action, uint8_t mode)
{
    switch (mode)
    {
    case GENERAL:
        /* S1边沿读取 */
        *s1_action = Change_from_middle(S1);
        if (*s1_action == NO_ACTION)
            *s1_action = Change_to_middle(S1);

        /* S2边沿读取 */
        *s2_action = Change_from_middle(S2);
        if (*s2_action == NO_ACTION)
            *s2_action = Change_to_middle(S2);

        /* S2状态读取 */
        if (RC_data.Remote_Data.s2 == 1)
        { // S2在上-正常二代自瞄
            Aimbot_FreshMouseClick(1);
            ComputerCTRL_EN = 0;
        }
        else if (RC_data.Remote_Data.s2 == 3)
        { // S2在中-强行不自瞄
            ComputerCTRL_EN = 0;
            Aimbot_LoosenCallback(); // 退出自瞄
        }
        else if (RC_data.Remote_Data.s2 == 2)
            // S2在下-客户端控制
            ComputerCTRL_EN = 1;

        // 进入和退出遥控器模式的时候默认不跟随
        if (*s2_action == middle_to_down)
            MotionModeSet_Callback(CHASSISMODE_KEY_MODE_NO_FOLLOW);
        else if (*s2_action == down_to_middle)
            MotionModeSet_Callback(CHASSISMODE_KEY_MODE_NO_FOLLOW);

        break;
    case REMOTE:
        if (*s1_action == middle_to_up) // 向上拨动
        {                               // 开关摩擦轮
            if (gunspeed == 0)
                Gun_RubEnable();
            else
                Gun_Disable();
        }
        else if (*s1_action == middle_to_down) //向下拨动
        {
            if (gunspeed == 0)
            { // 没有打开摩擦轮时，通过左侧拨杆向下拨动，可以调整底盘模式
                if (motion_mode == NO_FOLLOW)
                    MotionModeSet_Callback(CHASSISMODE_KEY_MODE_FOLLOW);
                else if (motion_mode == FOLLOW_GIMBAL)
                    MotionModeSet_Callback(CHASSISMODE_KEY_MODE_SMALL_GYRO);
                else if (motion_mode == SLOW_GYRO)
                    MotionModeSet_Callback(CHASSISMODE_KEY_MODE_FAST_GYRO);
                else if (motion_mode == FAST_GYRO)
                    MotionModeSet_Callback(CHASSISMODE_KEY_MODE_NO_FOLLOW);
                else
                    MotionModeSet_Callback(CHASSISMODE_KEY_MODE_NO_FOLLOW);
            }
        }

        if (*s2_action == middle_to_up) // 向上拨动
        {                               // 切换自瞄模式
            Aimbot_PressCallback();     // 进入自瞄
            if (Visual_Mode_Set == VISUAL_MODE_AIMBOT_V2)
#if defined CORE_USING_INFANTRY
                AimMode_Set_Callback(AIMMODE_SET_AIMBUFF_CONST_SPEED);
            else if (Visual_Mode_Set == VISUAL_MODE_AIMBUFF_CONST_SPEED)
                AimMode_Set_Callback(AIMMODE_SET_AIMBUFF_VARY_SPEED);
#elif defined CORE_USING_HERO
                AimMode_Set_Callback(AIMMODE_SET_ROTATING_OUTPOST);
            else if (Visual_Mode_Set == VISUAL_MODE_AIMBUFF_ROTATING_OUTPOST)
                AimMode_Set_Callback(AIMMODE_SET_STATIC_OUTPOST);
            else if (Visual_Mode_Set == VISUAL_MODE_AIMBUFF_STATIC_OUTPOST)
                AimMode_Set_Callback(AIMMODE_SET_OUTPOST_F);
#endif
            else
                AimMode_Set_Callback(AIMMODE_SET_AIMBOT);
        }
        break;
    case COMPUTER:
        // 客户端模式下可以修改自瞄颜色
        if (RC_data.Remote_Data.s1 == 1)
            // s1 在上修改为蓝色
            Color_Myself = My_Color_Blue;
        else if (RC_data.Remote_Data.s1 == 3)
            // s1 在中间修改为红色
            Color_Myself = My_Color_Red;
#ifndef CORE_USING_HERO
        if (*s1_action == middle_to_down) // 向下拨动
                                          // 开弹舱
            MiscSet_Callback(MISC_KEY_HATCH_OPEN);
        else if (*s1_action == down_to_middle)
            // 关弹舱
            MiscSet_Callback(MISC_KEY_HATCH_CLSE);
#endif
        break;
    default:
        break;
    }
    Write_Computer_Ctrl_Status(ComputerCTRL_EN); // 用于告知其他模块当前是否处于客户端模式
}

static uint16_t Remote_ResetCmd_Times = 0; // 复位指令持续的次数
/**
 * @brief 遥控器复位指令的检查
 * @author fwlh
 */
static void Remote_Reset_Check(void)
{
    // 不在客户端模式下遥控器不允许复位
    if (!ComputerCTRL_EN)
    {
        Remote_ResetCmd_Times = 0;
        return;
    }
    // 在客户端模式下需要遥控器拨到 起桨 的状态保持 200 次, 程序上是做的滞回比较
    if ((RC_data.Remote_Data.ch2 > 1550) && (RC_data.Remote_Data.ch3 < 500) &&
        (RC_data.Remote_Data.ch0 < 500) && (RC_data.Remote_Data.ch3 < 500))
    {
        if (Remote_ResetCmd_Times < 150)
            Remote_ResetCmd_Times += 1;
        else
        {
            // 复位全车单片机
            ResetCmd_Write(1);
            chassis_data_send(); // 由于发送数据是在本线程中进行的, 所以需要立即调用一次发送
            Robot_Reset_Gimbal();
        }
    }
    else
    {
        if (Remote_ResetCmd_Times > 2)
            Remote_ResetCmd_Times -= 2;
        else
            Remote_ResetCmd_Times = 0;
    }
}

#if defined CORE_USING_INFANTRY
static int Last_Motion_Mode = 0; // 用于记录上一次的底盘模式
static int Last_AimMode_Set = 0; // 用于记录上一次的自瞄模式
int8_t Last_RuneExit_Flag = 0;   // 用于记录上一次记录能量机关退出时的标志位
#endif
/**
* @brief：串口回调函数控制线程(14ms)
                时间片1，优先级1
* @param [in]	parameter:该参数不会被使用
* @return：		无
* @author：zzj
*/
static void RoboControl_entry(void *parameter)
{
    switch_action_e s1_action, s2_action; //动作触发临时变量
    rt_err_t res = RT_ERROR;
    SWDG_START(SWDG_ROBOCONTROL_ID);
    while (1)
    {
        /* 等待接收到遥控器数据 */
        res = rt_sem_take(&RoboControl_sem, 50);
        // 判断本次线程唤醒是由于 超时 或 收到信号量
        if (res == RT_EOK)
        {
            /* 按键状态读取 */
            RC_Key_Process();

            gunspeed = Rub_speed_ReadSet();

            /* 处理 s1 和 s2 按键的通用动作并做相应的动作或者模式更改 */
            General_Key_Action_Process(&s1_action, &s2_action, GENERAL);
            // 遥控器复位指令检查
            Remote_Reset_Check();

            if (ComputerCTRL_EN == 1)
            {
                // 客户端控制模式
                General_Key_Action_Process(&s1_action, &s2_action, COMPUTER);
                Computer_CTR(&RC_data);
            }
            else
            {
                FillMode_EN = 0; // 遥控器模式，强制禁用鼠标控制底盘

                General_Key_Action_Process(&s1_action, &s2_action, REMOTE);

                Refresh_RemoteSet_Gimbal(); //读取并更新设定值
                /* 底盘控制 */
                Chassis_RemoteCTR(&RC_data);

                /* 开火指令 */
                if ((RC_data.Remote_Data.s1 == 2) && (gunspeed != 0 && RC_data.Remote_Data.s2 != 2))
                    // 弹速设定值不为 0 且没有开启客户端模式, 开火(遥控器自瞄模式下可以视觉自动开火, 该开火指令与遥控器开火指令并行)
                    Gun_FireSet(FIRE_ON);
                else
                    Gun_FireSet(FIRE_OFF);
            }

#if defined CORE_USING_INFANTRY
            // 记录能量机关退出的相关标志位
            if ((!Exit_AimbotFlag) && ((Visual_Mode_Set == VISUAL_MODE_AIMBUFF_CONST_SPEED) || (Visual_Mode_Set == VISUAL_MODE_AIMBUFF_VARY_SPEED)) &&
                (Last_RuneExit_Flag != VisualFlag_ExitRune))
                AimMode_Set_Callback(AIMMODE_SET_AIMBOT);
            Last_RuneExit_Flag = VisualFlag_ExitRune;
            // 退出能量机关时将底盘模式修改回进入之前的模式
            if ((!Exit_AimbotFlag) && (Last_AimMode_Set != Visual_Mode_Set) &&
                ((Last_AimMode_Set == VISUAL_MODE_AIMBUFF_CONST_SPEED) || (Last_AimMode_Set == VISUAL_MODE_AIMBUFF_VARY_SPEED)))
                motion_mode = Last_Motion_Mode;
            Last_AimMode_Set = Visual_Mode_Set;
            Last_Motion_Mode = motion_mode;
#endif
        }
        SWDG_FEED(SWDG_ROBOCONTROL_ID);
    }
}

/**
 * @brief：RoboControl初始化
 * @param [in]	无
 * @return：		无
 * @author：zzj
 */
rt_err_t RoboControl_init(void)
{
    ModCTR_Init();
    //将移动用的按键的去抖动设置为1次，提高移动过程响应速度
    Key_SetPressConfirm(FOREWORD_KEY, 1);
    Key_SetPressConfirm(BACK_KEY, 1);
    Key_SetPressConfirm(LEFT_KEY, 1);
    Key_SetPressConfirm(RIGHT_KEY, 1);

    /* 遥控器数据处理函数创建 */
    rt_thread_t thread = rt_thread_create("RoboCtrl", RoboControl_entry, RT_NULL, 2048, THREAD_PRIO_ROBOCONTROL, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    return RT_EOK;
}
