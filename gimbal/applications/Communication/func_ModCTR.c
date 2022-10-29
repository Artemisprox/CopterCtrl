#include <func_ModCTR.h>
#include <drv_GimbalPublic.h>
#include <func_Key_Record.h>
#include "func_KeyCallback.h"
#include "drv_UI_Flags.h"
#include "drv_Key_Set.h"
#include "drv_utils.h"
#include "func_gun.h"
#include "drv_canthread.h"
#include "drv_ExactSmooth.h"
#include "func_GimbalSet.h"
#include "mod_gimbal.h"
#include "drv_IMU.h"
#include "func_gun.h"

static rt_int16_t basespeed = 1300; // 底盘速度设定值
static rt_int16_t Shift_SpeedUp = 0;
static rt_int16_t Ctrl_SpeedDown = 0;

/* 在按键回调函数里面修改的变量 */
rt_int8_t motion_mode = 0;                      // 底盘运动模式
static rt_uint8_t probe_last_motion_mode = 0;   // 探头模式结束以后该设定的底盘模式
float SetYaw_NotViewing;                        // 在进入探头模式之前的 Yaw 轴设定值
static rt_uint8_t moveback_end_motion_mode = 0; // 回头模式结束以后该设定的底盘模式

static rt_tick_t Recover_Start_Tick = 0; // 开始从探头模式恢复/一键回头的时刻
static rt_uint8_t Recover_Times = 0;     // 用于探头模式/一键回头检查控制精度的连续比较计数变量
static rt_uint8_t Probe_Back_Flag = 0;   // 处于探头模式恢复状态的标志位
static rt_uint8_t Moving_Back_Flag = 0;  // 处于一键回头模式的标志位

rt_uint8_t GimbalSetPower = 0; // 云台给底盘的设定功率

rt_uint8_t In_Dangling_Mode = 0; // 记录上一次是否处于吊射模式中
rt_int8_t Dangling_Motion_Mode;  // 进入吊射模式时的底盘模式

rt_int8_t FillMode_EN = 0; // 补弹模式标志位

float PitchFix = 0;
float YawFix = 0;

mixed_msg_t mixedMsgs = {0};

static int Robot_Chassis_Reset;

// 设置底盘复位命令
void ResetCmd_Write(int Reset_Flag)
{
    Robot_Chassis_Reset = Reset_Flag;
}

// 本文件内模块读取底盘复位命令
static int ResetCmd_Read(void)
{
    int cmd = Robot_Chassis_Reset;
    Robot_Chassis_Reset = 0;
    return cmd;
}

static uint8_t Now_In_Probe = 0; // 记录当前是不是在探头模式中
// 进入/退出探头模式需要调用的函数
void Enter_Probe_Mode(int Enter)
{
    if (Enter)
    {
        // 探头模式无法重复进入
        if (Now_In_Probe)
            return;
        Now_In_Probe = (uint8_t)Enter;
        // 进入探头模式之前需要记录现在的 Yaw 轴设定值
        SetYaw_NotViewing = Read_Real_Set(Yaw_Set);
        // 记录当前底盘模式
        probe_last_motion_mode = motion_mode;
        // 底盘模式修改为底盘自主
        motion_mode = CHASS_AUTO;
    }
    else
    {
        // 未进入探头模式时不能退出探头模式
        if (!Now_In_Probe)
            return;
        Now_In_Probe = (uint8_t)Enter;
        // 把 Yaw 轴设定值改回去
        Smooth_SetDataABS(&Smooth_YawAngleSet, SetYaw_NotViewing);
        // 延时把底盘模式该回去
        Probe_Back_Flag = 1;
        Recover_Start_Tick = rt_tick_get();
    }
    // 清空计数变量
    Recover_Times = 0;
}

// 进入一键回头模式时调用的函数
void Enter_MoveBack_Mode(void)
{
    Recover_Start_Tick = rt_tick_get();
    Moving_Back_Flag = 1;
    Smooth_SetDataADD(&Smooth_YawAngleSet, 180.f);
    if (motion_mode == FOLLOW_GIMBAL)
        moveback_end_motion_mode = FOLLOWBACK_GIMBAL;
    else if (motion_mode == FOLLOWBACK_GIMBAL)
        moveback_end_motion_mode = FOLLOW_GIMBAL;
    else
        moveback_end_motion_mode = motion_mode;
    motion_mode = MOVE_BACK; // 倒车模式
    // 清空计数变量
    Recover_Times = 0;
}

// 记录进入吊射模式时的底盘模式
void Dangling_RecNow_MotionMode(void)
{
    Dangling_Motion_Mode = motion_mode;
}

void Refresh_ComputerSet_Gimbal(void);
void Chassis_ComputerCTR(RC_Ctrl_t *RC_Data_in);
/**
 * @brief：客户端控制主函数
 * @param [RC_Ctrl_t*] RC_Data_in：遥控器数据结构体
 * @return：		无
 * @author：zzj
 */
void Computer_CTR(RC_Ctrl_t *RC_Data_in)
{
#if defined CORE_USING_HERO
    // 从吊射模式恢复
    if (!Read_Dangling_Mode() && In_Dangling_Mode)
        motion_mode = Dangling_Motion_Mode;
    In_Dangling_Mode = Read_Dangling_Mode();
#endif /* CORE_USING_HERO */
    // 探头模式或者一键回头模式的相关底盘模式的恢复
    if (Probe_Back_Flag || Moving_Back_Flag)
    {
        // 经历的时间比较长(0.8s)或者控制误差比较小以后可以把底盘模式修改回去
        if (rt_tick_get() - Recover_Start_Tick > 800)
        {
            // 退出恢复等待状态
            if (Probe_Back_Flag)
            {
                motion_mode = probe_last_motion_mode;
                Probe_Back_Flag = 0;
            }
            else
            {
                motion_mode = moveback_end_motion_mode;
                Moving_Back_Flag = 0;
            }
        }
        // 进行控制精度的检查
        else if (fabsf(CtrlErr_Yaw) < 20.f)
        {
            if (Recover_Times < 200)
                ++Recover_Times;
        }
        else if (Recover_Times > 2)
            Recover_Times -= 2;
        else
            Recover_Times = 0;
        if (Recover_Times > 25)
        {
            // 退出恢复等待状态
            if (Probe_Back_Flag)
            {
                motion_mode = probe_last_motion_mode;
                Probe_Back_Flag = 0;
            }
            else if(Moving_Back_Flag)
            {
                motion_mode = moveback_end_motion_mode;
                Moving_Back_Flag = 0;
            }
        }
    }

    /* 云台控制 */
    Refresh_ComputerSet_Gimbal(); //更新电脑端云台数据

    /* 发弹, 只有在此时自瞄开火指令才能生效 */
    if (Key_GetState(MOUSE_L) == 1)
    {
        if ((abs(Read_Gun_Motor(RubMotorLeft)->dji.speed) > 1000) && (Rub_speed_ReadSet() > 0))
            // 保证摩擦轮已经正常转动
            Gun_FireSet(FIRE_ON);
        else
        {
            Gun_FireSet(FIRE_OFF);
            if (Rub_speed_ReadSet() == 0)
                Gun_RubEnable();
        }
    }
    else
        Gun_FireSet(FIRE_OFF);

    /* 发送枪管相关数据 */
    // strike_status_send(&gun1);
    chassis_data_send();

    /* Shift加速 */
    if (Key_GetState(ACCL_KEY) == 1)
        Shift_SpeedUp = 1;
    else
        Shift_SpeedUp = 0;

    /* Ctrl减速 */
    if (Key_GetState(SLOW_KEY) == 1)
        Ctrl_SpeedDown = 1;
    else
        Ctrl_SpeedDown = 0;

    /* 底盘运动控制 */
    Chassis_ComputerCTR(RC_Data_in);
}

/**
 * @brief：读取并更新遥控器云台设定值
 * @param [in]	无
 * @return：		无
 * @author：zzj
 */
void Refresh_RemoteSet_Gimbal()
{
    static float yaw_add = 0;
    static float pitch_add = 0;
    //读取增量
    yaw_add = -(RC_data.Remote_Data.ch0 - 1024) * 0.009f;
    pitch_add = (RC_data.Remote_Data.ch1 - 1024) * 0.004f;
    //发送设定值增量
    Smooth_SetDataADD(&Smooth_PitchAngleSet, pitch_add);
    Smooth_SetDataADD(&Smooth_YawAngleSet, yaw_add);
}

/**
 * @brief：读取并更新电脑端云台设定值
 * @param [in]	无
 * @return：		无
 * @author：zzj
 */
void Refresh_ComputerSet_Gimbal(void)
{
    static float yaw_add = 0;
    static float pitch_add = 0;
    static int16_t xspeed_last = 0;
    static int16_t yspeed_last = 0;

    //错误数据处理
    if (abs(RC_data.Mouse_Data.x_speed) > 20000)
        RC_data.Mouse_Data.x_speed = xspeed_last;
    else
        xspeed_last = RC_data.Mouse_Data.x_speed;
    if (abs(RC_data.Mouse_Data.y_speed) > 20000)
        RC_data.Mouse_Data.y_speed = yspeed_last;
    else
        yspeed_last = RC_data.Mouse_Data.y_speed;

    yaw_add = -RC_data.Mouse_Data.x_speed * MOUSE_SPEED_GAIN_YAW;
    pitch_add = -RC_data.Mouse_Data.y_speed * MOUSE_SPEED_GAIN_PITCH;
#if defined CORE_USING_HERO
    // 在吊射模式中就需要大幅度衰减鼠标移动命令
    if (Read_Dangling_Mode())
    {
        yaw_add *= MOUSE_SPEED_EXTRAGAIN_DANGLING;
        pitch_add *= MOUSE_SPEED_EXTRAGAIN_DANGLING;
    }
#endif /* CORE_USING_HERO */
    if ((Gimbal_SetData_Out.State == RT_EOK) && (Exit_AimbotFlag == 0))
    { // 自瞄且有目标
        PitchFix += pitch_add * AIMBOT_FIX_GAIN;
        YawFix += yaw_add * AIMBOT_FIX_GAIN;
    }
    else if (Exit_AimbotFlag == 0)
    { // 自瞄无目标
    }
    else
    { // 非自瞄，清空积分
        //读取增量
        PitchFix = 0;
        YawFix = 0;
    }

    //发送设定值增量
    if (FillMode_EN == 0)
    {
        Smooth_SetDataADD(&Smooth_PitchAngleSet, pitch_add);
        Smooth_SetDataADD(&Smooth_YawAngleSet, yaw_add);
    }
    else
    {
        // 补弹模式下，不修改云台设定值
        Smooth_SetDataADD(&Smooth_PitchAngleSet, 0);
        Smooth_SetDataADD(&Smooth_YawAngleSet, 0);
    }
}

/**
 * @brief  获取并发送底盘控制结构体
 * @param  xspeed：x轴速度
 * @param  yspeed：y轴速度
 * @param  mode: 模式（不跟随0，跟随1，独立底盘2）
 * @param  angel_or_speed：跟随时的角度，不跟随时速度
 */
static void chassis_ctl(rt_int16_t xspeed, rt_int16_t yspeed, rt_uint8_t mode, rt_int16_t angel_or_speed)
{
    struct rt_can_msg txmsg;

    txmsg.id = CHASSIS_CTL;
    txmsg.ide = RT_CAN_STDID;
    txmsg.rtr = RT_CAN_DTR;
    txmsg.len = 8;
    txmsg.data[0] = (rt_uint8_t)(xspeed >> 8);
    txmsg.data[1] = (rt_uint8_t)(xspeed);
    txmsg.data[2] = (rt_uint8_t)(yspeed >> 8);
    txmsg.data[3] = (rt_uint8_t)(yspeed);
    txmsg.data[4] = (rt_uint8_t)(angel_or_speed >> 8);
    txmsg.data[5] = (rt_uint8_t)(angel_or_speed);
    txmsg.data[6] = (rt_uint8_t)(mode);
    txmsg.data[7] = GimbalSetPower;
    rt_device_write(can1_dev, 0, &txmsg, sizeof(txmsg));
}

static uint8_t ComputerCtrl_Flag = 0; // 标志当前是否处于客户端控制模式
void Write_Computer_Ctrl_Status(int Now_Computer_Ctrl)
{
    ComputerCtrl_Flag = Now_Computer_Ctrl;
}

rt_sem_t chassis_send_sem = RT_NULL; // 该信号量用于加锁, 防止访问冲突
/***
 * @brief    向底盘发送各类设定数据
 * @param
 * @retval   none
 * @author   dxy
 ***/
void chassis_data_send(void)
{
    // 访问冲突时不等待直接返回
    if (rt_sem_trytake(chassis_send_sem) != RT_EOK)
        return;
    static uint16_t cnt = 0;
    struct rt_can_msg txmsg;
    rt_tick_t TickNow = rt_tick_get();

    mixedMsgs.ui_reset = ui_reset;
    mixedMsgs.chassis_reset = ResetCmd_Read();
    mixedMsgs.current_menu = current_menu;
    mixedMsgs.motion_mode = motion_mode;
    mixedMsgs.now_viewing = Now_In_Probe;
    mixedMsgs.strike_mode = Read_Now_Gun_Mode();
    mixedMsgs.magazine_status = magazine_state;
    mixedMsgs.heatlimit_status = heatlimit_state;
    mixedMsgs.aimbot_mode = ui_aimbot_mode;
    mixedMsgs.self_color = Color_Myself;
    mixedMsgs.power_restrictions_lim = power_restrictions_open_flag;
    mixedMsgs.rub_started = Read_Rub_Started();
    mixedMsgs.now_client_control = ComputerCtrl_Flag;
    mixedMsgs.capacity_close_flag = capacity_close;
    mixedMsgs.force_refsystem_offline = force_refsystem_offline;
    mixedMsgs.set_chassis_mode = (chassis_mode ? chassis_mode - Chassis_Mode_Start : 0);
    mixedMsgs.set_ammobooster_mode = (ammobooster_mode ? ammobooster_mode - AmmoBooster_Mode_Start : 0);
    mixedMsgs.set_level = level;
    mixedMsgs.tick_now = utils_max_2_int((uint8_t)(TickNow / 100), 255);
    mixedMsgs.visual_com_online = ((TickNow - Visual_LastFresh_Tick < 200) ? 1 : 0);
    mixedMsgs.visual_working_correct = VisualFlag_WorkingCorrect;
    mixedMsgs.yaw_motor_online = ((TickNow - Yaw.dji.FreshTick < 200) ? 1 : 0);
    mixedMsgs.pitch_motor_online = ((TickNow - Pitch.dji.FreshTick < 200) ? 1 : 0);
    mixedMsgs.right_rub_motor_online = ((TickNow - Read_Gun_Motor(RubMotorRight)->dji.FreshTick < 200) ? 1 : 0);
    mixedMsgs.left_rub_motor_online = ((TickNow - Read_Gun_Motor(RubMotorLeft)->dji.FreshTick < 200) ? 1 : 0);
    mixedMsgs.launch_motor_online = ((TickNow - Read_Gun_Motor(LaunchMotor)->dji.FreshTick < 200) ? 1 : 0);
    mixedMsgs.strike_stuck = (Read_Strike_Motor_Stuck_Status() ? 1 : 0);

    txmsg.id = CHASSIS_DATA;
    txmsg.ide = RT_CAN_STDID;
    txmsg.rtr = RT_CAN_DTR;
    txmsg.len = 8;
    rt_memcpy(txmsg.data, (void *)&mixedMsgs, 8);
    rt_device_write(can1_dev, 0, &txmsg, sizeof(txmsg));
    if (cnt % 3 == 0)
    {
        txmsg.id = CHASSIS_IMU_DATA;
        txmsg.data[0] = ((rt_uint8_t *)(&gimbal_atti.pitch))[0];
        txmsg.data[1] = ((rt_uint8_t *)(&gimbal_atti.pitch))[1];
        txmsg.data[2] = ((rt_uint8_t *)(&gimbal_atti.pitch))[2];
        txmsg.data[3] = ((rt_uint8_t *)(&gimbal_atti.pitch))[3];
        txmsg.data[4] = ((rt_uint8_t *)(&gimbal_atti.yaw))[0];
        txmsg.data[5] = ((rt_uint8_t *)(&gimbal_atti.yaw))[1];
        txmsg.data[6] = ((rt_uint8_t *)(&gimbal_atti.yaw))[2];
        txmsg.data[7] = ((rt_uint8_t *)(&gimbal_atti.yaw))[3];
        rt_device_write(can1_dev, 0, &txmsg, sizeof(txmsg));
    }
    cnt++;
    if (cnt > 3000)
        cnt = 0; // 防溢出保护
    // 访问结束, 解锁
    rt_sem_release(chassis_send_sem);
}

/**
 * @brief：遥控器控制底盘函数
 * @param [RC_Ctrl_t*] RC_Data_in：遥控器数据结构体
 * @return：		无
 * @author：zzj
 */
void Chassis_RemoteCTR(RC_Ctrl_t *RC_Data_in)
{
    // 发送 IMU 等数据
    chassis_data_send();
    // 发送底盘控制信息
    switch (motion_mode)
    {
    case NO_FOLLOW: //非跟随
        chassis_ctl((RC_Data_in->Remote_Data.ch2 - 1024) * 4, (RC_Data_in->Remote_Data.ch3 - 1024) * 4, 0, 0);
        break;
    case FOLLOW_GIMBAL: //跟随
        chassis_ctl((RC_Data_in->Remote_Data.ch2 - 1024) * 6, (RC_Data_in->Remote_Data.ch3 - 1024) * 6, 1, 0);
        break;
    case SLOW_GYRO: //慢陀螺
        chassis_ctl((RC_Data_in->Remote_Data.ch2 - 1024) * 3, (RC_Data_in->Remote_Data.ch3 - 1024) * 3, 0, SMALLGYRO_ROTATE_SPEED);
        break;
    case FAST_GYRO: //快陀螺
    {
        rt_int16_t xspeed_out = (RC_Data_in->Remote_Data.ch2 - 1024) * 2;
        rt_int16_t yspeed_out = (RC_Data_in->Remote_Data.ch3 - 1024) * 2;
        if (sqrtf(xspeed_out * xspeed_out + yspeed_out * yspeed_out) > basespeed * 1.2f)
            chassis_ctl(xspeed_out, yspeed_out, 0, (rt_int16_t)utils_map(0.2f, 0.f, 1.f, SMALLGYRO_ROTATE_SPEED, FASTGYRO_ROTATE_SPEED));
        else
            chassis_ctl(xspeed_out, yspeed_out, 0, (FASTGYRO_ROTATE_SPEED + (int)(FASTGYRO_ROTATE_CHANGE_A * sinf(rt_tick_get() * FASTGYRO_RATATE_CHANGE_W))));
        break;
    }
    default:
        chassis_ctl(0, 0, 0, 0);
        break;
    }
}

/**
 * @brief：客户端控制底盘函数
 * @param [RC_Ctrl_t*] RC_Data_in：遥控器数据结构体
 * @return：		无
 * @author：zzj
 */
static rt_int16_t xspeed_out = 0; //发送给底盘的xspeed
static rt_int16_t yspeed_out = 0; //发送给底盘的yspeed

ExactSmth_CTRL_S MouseX_Filter, MouseY_Filter;

static volatile float Mouse_Speed_K_use = MOUSE_SPEED_K;

void Chassis_ComputerCTR(RC_Ctrl_t *RC_Data_in)
{
    float x_set = 0; //记录AD按键状态的临时变量
    float y_set = 0; //记录WS按键状态的临时变量

    Smooth_SetDataADD(&MouseX_Filter, RC_data.Mouse_Data.x_speed);
    Smooth_SetDataADD(&MouseY_Filter, -RC_data.Mouse_Data.y_speed);

    if (FillMode_EN == 0)
    {
        x_set = Key_GetState(RIGHT_KEY) - Key_GetState(LEFT_KEY);
        y_set = Key_GetState(FOREWORD_KEY) - Key_GetState(BACK_KEY);
        if (Shift_SpeedUp)
        {
            xspeed_out = (rt_int16_t)(basespeed * x_set * SHIFT_UP);
            yspeed_out = (rt_int16_t)(basespeed * y_set * SHIFT_UP);
        }
        else if (Ctrl_SpeedDown)
        {
            xspeed_out = (rt_int16_t)(basespeed * x_set * CTRL_LOW);
            yspeed_out = (rt_int16_t)(basespeed * y_set * CTRL_LOW);
        }
        else
        {
            xspeed_out = (rt_int16_t)(basespeed * x_set);
            yspeed_out = (rt_int16_t)(basespeed * y_set);
        }
        Smooth_SetData_Restart(&MouseX_Filter, 0);
        Smooth_SetData_Restart(&MouseY_Filter, 0);
    }
    else
    {
        Smooth_GetDataADD(&x_set, &MouseX_Filter);
        Smooth_GetDataADD(&y_set, &MouseY_Filter);
        xspeed_out = (rt_int16_t)(Mouse_Speed_K_use * x_set);
        yspeed_out = (rt_int16_t)(Mouse_Speed_K_use * y_set);
    }

    switch (motion_mode)
    {
    case NO_FOLLOW: //非跟随
        chassis_ctl(xspeed_out, yspeed_out, 0, 0);
        break;
    case FOLLOW_GIMBAL: //跟随
        chassis_ctl(xspeed_out, yspeed_out, 1, 0);
        break;
    case FOLLOWBACK_GIMBAL: // 反向跟随
        chassis_ctl(xspeed_out, yspeed_out, 1, 4096);
        break;
    case SLOW_GYRO: //慢陀螺
        chassis_ctl(xspeed_out, yspeed_out, 0, SMALLGYRO_ROTATE_SPEED);
        break;
    case FAST_GYRO:
        if (sqrtf(xspeed_out * xspeed_out + yspeed_out * yspeed_out) > basespeed * 1.2f)
            chassis_ctl(xspeed_out, yspeed_out, 0, (rt_int16_t)utils_map(0.2f, 0.f, 1.f, SMALLGYRO_ROTATE_SPEED, FASTGYRO_ROTATE_SPEED));
        else
            chassis_ctl(xspeed_out, yspeed_out, 0, (FASTGYRO_ROTATE_SPEED + (int)(FASTGYRO_ROTATE_CHANGE_A * sinf(rt_tick_get() * FASTGYRO_RATATE_CHANGE_W))));
        break;
    case MOVE_BACK: //倒车模式
        chassis_ctl(-xspeed_out, -yspeed_out, 2, 0);
        break;
    case CHASS_AUTO: // 底盘自主
        // 探头模式下需要判断之前是不是在反向跟随
        if (Now_In_Probe && (probe_last_motion_mode == FOLLOWBACK_GIMBAL))
            chassis_ctl(-xspeed_out, -yspeed_out, 2, 0);
        else
            chassis_ctl(xspeed_out, yspeed_out, 2, 0);
        break;
    default:
        chassis_ctl(0, 0, 0, 0);
        break;
    }
}

void ModCTR_Init(void)
{
    Smooth_Init(&MouseX_Filter, 0, 30);
    Smooth_Init(&MouseY_Filter, 0, 30);

    chassis_send_sem = rt_sem_create("chassis send", 1, RT_IPC_FLAG_FIFO);
}
