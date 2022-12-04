#include "mod_gimbal.h"

#include "func_GimbalSet.h"
#include "func_GimbalPID.h"
#include "func_GimbalFB.h"
#include "func_bmi088.h"
#include "drv_StrikeMotor.h"
#include "robodata.h"
#include "drv_remote.h"
#include "mod_aimbot_V2.h"
#include "drv_Queue.h"
#include "drv_thread.h"
#include "func_gun.h"
#include "mod_Monitor.h"
#include "drv_utils.h"
#include "drv_canthread.h"
#include "drv_Aimbot_Public.h"
#include <stdbool.h>

//云台电机PID线程句柄
static rt_thread_t gimbal_control = RT_NULL;

// 电机数据结构体
Motor_t Yaw;
Motor_t Pitch;

float CtrlErr_Pitch = 0, CtrlErr_Yaw = 0; // Pitch 和 Yaw 的控制误差

#if defined CAR_USING_LINK
float Connect_Alpha;  // 用于连杆计算的角度
float Pitch_F0_Use;   // Pitch重心在转轴正前方时产生的扭矩
float Pitch_ANG0_Use; // Pitch重心在转轴正上方时云台姿态角
float DeltaEncoderAngle;
float CompenOutGain;
float pitch_gain;
#endif

GimbalCTRL_Set_Type SetAng;                      // 大地坐标系下的云台姿态设定值
static AttitudeData_Type GroundFrame_GimbalAtti; // 大地坐标系下的云台姿态
static AttitudeData_Type MotorFrame_GimbalAtti;  // 电机坐标系下的云台姿态
static AttitudeData_Type MotorFrame_GimbalErr;   // 电机坐标系下的云台姿态误差
/**
 * @brief 将大地坐标系下的云台角度设定值与反馈值换算为电机坐标系下的相关数据
 * @author fwlh
 * @param  SetAng                大地坐标系下的云台角度设定值
 * @param  GroundAtti            大地坐标系下的云台当前姿态
 * @param  MotorAtti             电机坐标系下的云台当前姿态
 * @param  MotorFrame_GimbalErr  电机坐标系下的角度误差
 */
static void GimbalSet_Gound_To_Motor(GimbalCTRL_Set_Type *const *SetAng,
                                     AttitudeData_Type *const *GroundAtti,
                                     AttitudeData_Type *const *MotorAtti,
                                     AttitudeData_Type *MotorFrame_GimbalErr)
{
    // 注意 HERO 战队机器人普遍采用左手系描述位姿(x 轴正方向向右, y 轴正方向向后, z 轴正方向向上)
}

static struct rt_timer task_1ms;
static struct rt_semaphore gimbal_1ms_sem; //定时器超时函数发送的1ms信号量

static rt_uint8_t GimbalMotor_Valid = 1;    // 用于判断该时刻是否该给云台电机发送电流, 置 0 会停止发送
static rt_sem_t CtrlErr_Calc_Sem = RT_NULL; //  用于防止控制误差计算时函数被重入

volatile float JSCOPE_GIMBALTimeLag;
// 用于记录调用的时间间隔
static void Frq_Rec(void)
{
    static int LastTime = 0;
    static int count = 0;
    int TimeNow;
    count++;
    if (count >= 10)
    { // 10次调用后结算一次平均时延
        TimeNow = rt_tick_get();
        JSCOPE_GIMBALTimeLag = (TimeNow - LastTime) / 10.0f;
        LastTime = TimeNow;
        count = 0;
    }
}

static float LastPitchFB, LastYawFB;   // 上一次的角度反馈数值
static float TempPitchErr, TempYawErr; // 计算控制误差的中间变量
static float AngleExtended;            // 展开以后的角度
static bool LastErr_Nan_Flag = false;  // 用于标记上次控制误差计算出了 Nan, 下次计算时默认上次数据取 0
// 更新控制误差数据
void Ctrl_Err_Cal(float SetPitchAng, float SetYawAng)
{
    // 检查信号量是否被初始化
    if (CtrlErr_Calc_Sem)
    {
        // 防止函数被重入
        if (rt_sem_trytake(CtrlErr_Calc_Sem) != RT_EOK)
            return;
        while (rt_sem_trytake(CtrlErr_Calc_Sem) == RT_EOK)
            continue;
        // 输入数据合法性检查
        if (isnan(SetPitchAng) || isnan(SetYawAng))
        {
            CtrlErr_Pitch = 1800.f; // 出错时输出 误差=1800
            CtrlErr_Yaw = 1800.f;   // 出错时输出 误差=1800
            LastErr_Nan_Flag = true;
            rt_sem_release(CtrlErr_Calc_Sem);
            return;
        }
        // 真实角度先展开再滞后滤波
        AngleExtended = utils_angle_difference(GimbalFB.FB_This.Pitch, LastPitchFB) + LastPitchFB;
        LastPitchFB = UTILS_LP_FAST(GimbalFB.FB_This.Pitch, LastPitchFB, 0.5f);
        AngleExtended = utils_angle_difference(GimbalFB.FB_This.Yaw, LastYawFB) + LastYawFB;
        LastYawFB = UTILS_LP_FAST(GimbalFB.FB_This.Yaw, LastYawFB, 0.5f);
        // 误差计算
        TempPitchErr = utils_angle_difference(SetPitchAng, LastPitchFB);
        TempYawErr = utils_angle_difference(SetYawAng, LastYawFB);
        // // 两次计算过程误差的变化不能超过 180 度, 注意要判断上次的数据异常标志位
        // if (LastErr_Nan_Flag)
        // {
        //     // 上次数据异常时认为上次的数据是 0, 主要是考虑误差变化限幅
        //     CtrlErr_Pitch = 0.f;
        //     CtrlErr_Yaw = 0.f;
        //     LastErr_Nan_Flag = false; // 清空标志位
        // }
        // utils_norm_circle_number(&TempPitchErr, CtrlErr_Pitch - 180.f, 360.f);
        // utils_norm_circle_number(&TempYawErr, CtrlErr_Yaw - 180.f, 360.f);
        // 控制误差不能超过 -180 或者 180
        utils_norm_circle_number(&TempPitchErr, -180.f, 360.f);
        utils_norm_circle_number(&TempYawErr, -180.f, 360.f);
        // 将计算的误差赋值为对外输出的控制误差
        CtrlErr_Pitch = TempPitchErr;
        CtrlErr_Yaw = CtrlErr_Yaw;
        // 释放信号量允许下一次运行
        rt_sem_release(CtrlErr_Calc_Sem);
    }
    // 防止编译器报 warning
    (void)LastErr_Nan_Flag;
    (void)AngleExtended;
}

// 默认开启云台电机堵转保护
#define GIMBALSTUCK_PROTECT 1

#if (GIMBALSTUCK_PROTECT)
static int GimbalStuckCount = 0;
static int GimbalStuckFlag = 0;
#endif

static float PitchSpeedFB, YawSpeedFB;
#if defined CORE_USING_HERO
StrikeMotor_CtrlData_s StrikeMotor_CtrlData;
#endif

// 控制云台电机的 CAN 报文结构体
#if defined CORE_USING_INFANTRY
static struct rt_can_msg gimctl_msg;
#elif defined CORE_USING_HERO
static struct rt_can_msg gimctl_msg[2];
#endif
// 云台控制计算
static void Gimbal_Controller_Run(void)
{
    // 如果云台电机失能会直接停止控制器的计算
    if (GimbalMotor_Valid)
    {
        /* PID */
        if (GimbalFB.FBS_Now_Pitch == FB_ENCD)
        {
            // PitchSpeedFB = PitchMotorSymbol * Pitch.dji.speed * 6.0f; // 注意换算单位
            PitchSpeedFB = gimbal_atti.pitch_speed * 0.5f + PitchSpeedFB * 0.5f;
            // PitchSpeedFB = 0;
            Gimbal_Pitch_PID_RUN(&Pitch, PitchSpeedFB, GimbalFB.FB_This.Pitch, 0.1f);
        }
        else
        {
            PitchSpeedFB = gimbal_atti.pitch_speed * 0.9f + PitchSpeedFB * 0.1f;
#if defined CORE_USING_INFANTRY
            Gimbal_Pitch_PID_RUN(&Pitch, PitchSpeedFB, GimbalFB.FB_This.Pitch, 0.97f);
#elif defined CORE_USING_HERO
            Gimbal_Pitch_PID_RUN(&Pitch, PitchSpeedFB, GimbalFB.FB_This.Pitch, 0.7f);
#endif
        }

        if (GimbalFB.FBS_Now_Yaw == FB_ENCD)
        {
            // YawSpeedFB = YawMotorSymbol * Yaw.dji.speed * 6.0f; // 注意换算单位
            YawSpeedFB = gimbal_atti.yaw_speed * 0.05f + YawSpeedFB * 0.95f;
            // YawSpeedFB = 0;
            Gimbal_Yaw_PID_RUN(&Yaw, YawSpeedFB, GimbalFB.FB_This.Yaw, 0.f);
        }
        else
        {
            YawSpeedFB = gimbal_atti.yaw_speed * 0.9f + YawSpeedFB * 0.1f;
#if defined CORE_USING_INFANTRY
            Gimbal_Yaw_PID_RUN(&Yaw, YawSpeedFB, GimbalFB.FB_This.Yaw, 0.97f);
#elif defined CORE_USING_HERO
            Gimbal_Yaw_PID_RUN(&Yaw, YawSpeedFB, GimbalFB.FB_This.Yaw, 0.7f);
#endif
        }

#ifdef GIMBAL_BIAS_SET
        Pitch.spe.out += GIMBAL_BIAS_SET * cosf(gimbal_atti.pitch);
#endif

        // 6020 电机电流补偿
        Pitch.spe.out += Pitch.dji.speed * 70;

        //输出限幅
        if (Pitch.spe.out > Pitch.spe.out_limit_up)
            Pitch.spe.out = Pitch.spe.out_limit_up;
        else if (Pitch.spe.out < Pitch.spe.out_limit_down)
            Pitch.spe.out = Pitch.spe.out_limit_down;

#if (GIMBALSTUCK_PROTECT)
        if (GimbalStuckFlag == 0)
        {
            if ((fabsf(Pitch.spe.out) > 19000 && abs(Pitch.dji.speed) < 2) ||
                (fabsf(Yaw.spe.out) > 19000 && abs(Yaw.dji.speed) < 2))
            { // 疑似云台堵转
                if (RC_data.Mouse_Data.x_speed == 0 && RC_data.Mouse_Data.y_speed == 0)
                    GimbalStuckCount++;
                else
                    // 如果操作手在操作
                    GimbalStuckCount = 0;
            }
            else
                GimbalStuckCount = 0;
            if (GimbalStuckCount > 1500)
            {
                // 连续1S堵转触发保护
                GimbalStuckFlag = 1;
                GimbalStuckCount = 0;
            }
        }
        else
        { // 识别到云台堵转
            if (GimbalStuckCount < 5000 && RC_data.Mouse_Data.x_speed == 0 && RC_data.Mouse_Data.y_speed == 0)
            {
                // 堵转保护时间
                GimbalStuckCount++;
                Pitch.spe.out = 0;
                Yaw.spe.out = 0;
            }
            else
            { // 保护退出
                GimbalStuckCount = 0;
                GimbalStuckFlag = 0;
            }
        }
#endif

#if defined CORE_USING_INFANTRY
        /* 写电流值 */
        if ((rt_tick_get() - Pitch.dji.FreshTick < 100) && (Pitch.dji.FreshTick))
        {
            gimctl_msg.data[(rt_uint16_t)(PITCH_ID - 0x205) * 2] = (int)(Pitch.spe.out) >> 8;
            gimctl_msg.data[(rt_uint16_t)(PITCH_ID - 0x205) * 2 + 1] = (int)(Pitch.spe.out);
        }
        else
        {
            gimctl_msg.data[(rt_uint16_t)(PITCH_ID - 0x205) * 2] = 0;
            gimctl_msg.data[(rt_uint16_t)(PITCH_ID - 0x205) * 2 + 1] = 0;
        }
        if ((Yaw.dji.FreshTick) && (rt_tick_get() - Yaw.dji.FreshTick < 100))
        {
            gimctl_msg.data[(rt_uint16_t)(YAW_ID - 0x205) * 2] = (int)(Yaw.spe.out) >> 8;
            gimctl_msg.data[(rt_uint16_t)(YAW_ID - 0x205) * 2 + 1] = (int)(Yaw.spe.out);
        }
        else
        {
            gimctl_msg.data[(rt_uint16_t)(YAW_ID - 0x205) * 2] = 0;
            gimctl_msg.data[(rt_uint16_t)(YAW_ID - 0x205) * 2 + 1] = 0;
        }
// 如果存在第二个云台电机
#ifdef DUAL_PITCH_MOTOR
        gimctl_msg.data[(rt_uint16_t)(DUAL_PITCH_ID - 0x205) * 2] = (int)(-Pitch.spe.out) >> 8;
        gimctl_msg.data[(rt_uint16_t)(DUAL_PITCH_ID - 0x205) * 2 + 1] = (int)(-Pitch.spe.out);
#endif
        /* 发送电流值 */
        rt_device_write(can1_dev, 0, &gimctl_msg, sizeof(gimctl_msg));
        Ctrl_Err_Cal(Read_Real_Set(Pitch_Set), Read_Real_Set(Yaw_Set));
    }
    else
    {
        // 电机失能时全部发送 0
        gimctl_msg.data[(rt_uint16_t)(PITCH_ID - 0x205) * 2] = 0;
        gimctl_msg.data[(rt_uint16_t)(PITCH_ID - 0x205) * 2 + 1] = 0;
        gimctl_msg.data[(rt_uint16_t)(YAW_ID - 0x205) * 2] = 0;
        gimctl_msg.data[(rt_uint16_t)(YAW_ID - 0x205) * 2 + 1] = 0;
        rt_device_write(can1_dev, 0, &gimctl_msg, sizeof(gimctl_msg));
    }
#elif defined CORE_USING_HERO
        // 发射机构控制相关计算
        StrikeMotor_CtrlRoutine(&StrikeMotor_CtrlData);

        /* 写电流值 */
        gimctl_msg[0].data[(rt_uint16_t)(PITCH_ID - 0x205) * 2] = (rt_int16_t)(Pitch.spe.out) >> 8;
        gimctl_msg[0].data[(rt_uint16_t)(PITCH_ID - 0x205) * 2 + 1] = (rt_int16_t)(Pitch.spe.out);
        gimctl_msg[0].data[(rt_uint16_t)(YAW_ID - 0x205) * 2] = (rt_int16_t)(Yaw.spe.out) >> 8;
        gimctl_msg[0].data[(rt_uint16_t)(YAW_ID - 0x205) * 2 + 1] = (rt_int16_t)(Yaw.spe.out);
        gimctl_msg[0].data[(rt_uint16_t)(LAUNCH_ID - 0x205) * 2] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[LaunchMotor])) >> 8;
        gimctl_msg[0].data[(rt_uint16_t)(LAUNCH_ID - 0x205) * 2 + 1] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[LaunchMotor]));
// 如果存在第二个云台电机
#ifdef DUAL_PITCH_MOTOR
        gimctl_msg.data[(rt_uint16_t)(DUAL_PITCH_ID - 0x205) * 2] = (rt_int16_t)(-Pitch.spe.out) >> 8;
        gimctl_msg.data[(rt_uint16_t)(DUAL_PITCH_ID - 0x205) * 2 + 1] = (rt_int16_t)(-Pitch.spe.out);
#endif
        gimctl_msg[1].data[(rt_uint16_t)(ID_RUB_LEFT - 0x201) * 2] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[RubMotorLeft])) >> 8;
        gimctl_msg[1].data[(rt_uint16_t)(ID_RUB_LEFT - 0x201) * 2 + 1] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[RubMotorLeft]));
        gimctl_msg[1].data[(rt_uint16_t)(ID_RUB_RIGHT - 0x201) * 2] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[RubMotorRight])) >> 8;
        gimctl_msg[1].data[(rt_uint16_t)(ID_RUB_RIGHT - 0x201) * 2 + 1] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[RubMotorRight]));
        /* 发送电流值 */
#if (TEST_CLEAR_PID == 0)
        rt_device_write(can1_dev, 0, &gimctl_msg[0], sizeof(gimctl_msg[0]));
        rt_device_write(can2_dev, 0, &gimctl_msg[1], sizeof(gimctl_msg[1]));
#endif

        Ctrl_Err_Cal(Read_Real_Set(Pitch_Set), Read_Real_Set(Yaw_Set)); // 结算当前云台控制误差
    }
    else
    {
        // 电机失能时全部发送 0
        // 播弹叉相关计算
        StrikeMotor_CtrlRoutine(&StrikeMotor_CtrlData);

        /* 写电流值 */
        gimctl_msg[0].data[(rt_uint16_t)(PITCH_ID - 0x205) * 2] = 0;
        gimctl_msg[0].data[(rt_uint16_t)(PITCH_ID - 0x205) * 2 + 1] = 0;
        gimctl_msg[0].data[(rt_uint16_t)(YAW_ID - 0x205) * 2] = 0;
        gimctl_msg[0].data[(rt_uint16_t)(YAW_ID - 0x205) * 2 + 1] = 0;
        gimctl_msg[0].data[(rt_uint16_t)(LAUNCH_ID - 0x205) * 2] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[LaunchMotor])) >> 8;
        gimctl_msg[0].data[(rt_uint16_t)(LAUNCH_ID - 0x205) * 2 + 1] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[LaunchMotor]));
        // 如果存在第二个云台电机
#ifdef DUAL_PITCH_MOTOR
        gimctl_msg.data[(rt_uint16_t)(DUAL_PITCH_ID - 0x205) * 2] = 0;
        gimctl_msg.data[(rt_uint16_t)(DUAL_PITCH_ID - 0x205) * 2 + 1] = 0;
#endif
        gimctl_msg[1].data[(rt_uint16_t)(ID_RUB_LEFT - 0x201) * 2] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[RubMotorLeft])) >> 8;
        gimctl_msg[1].data[(rt_uint16_t)(ID_RUB_LEFT - 0x201) * 2 + 1] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[RubMotorLeft]));
        gimctl_msg[1].data[(rt_uint16_t)(ID_RUB_RIGHT - 0x201) * 2] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[RubMotorRight])) >> 8;
        gimctl_msg[1].data[(rt_uint16_t)(ID_RUB_RIGHT - 0x201) * 2 + 1] = ((rt_int16_t)(StrikeMotor_CtrlData.DataOut[RubMotorRight]));

        /* 发送电流值 */
#if (TEST_CLEAR_PID == 0)
        rt_device_write(can1_dev, 0, &gimctl_msg[0], sizeof(gimctl_msg[0]));
        rt_device_write(can2_dev, 0, &gimctl_msg[1], sizeof(gimctl_msg[1]));
#endif
    }
#endif
}

/**
 * @brief：云台线程
 * @param [in]	parameter:该参数不会被使用
 * @return：		无
 * @author：zzj
 */
static void Gimbal_control_thread(void *parameter)
{
#if defined CORE_USING_INFANTRY
    //初始化CAN控制帧
    gimctl_msg.id = GIMBAL_CTL;    //设置ID
    gimctl_msg.ide = RT_CAN_STDID; //标准帧
    gimctl_msg.rtr = RT_CAN_DTR;   //数据帧
    gimctl_msg.priv = 0;           //报文优先级最高
    gimctl_msg.len = 8;            //长度8

    //控制数据清零
    for (int a = 0; a < 8; a++)
        gimctl_msg.data[a] = 0;
#elif defined CORE_USING_HERO
    //初始化CAN控制帧
    gimctl_msg[0].id = GIMBAL_CTL;    //设置ID
    gimctl_msg[0].ide = RT_CAN_STDID; //标准帧
    gimctl_msg[0].rtr = RT_CAN_DTR;   //数据帧
    gimctl_msg[0].priv = 0;           //报文优先级最高
    gimctl_msg[0].len = 8;            //长度8

    gimctl_msg[1].id = STRIKE_ID;     //设置ID
    gimctl_msg[1].ide = RT_CAN_STDID; //标准帧
    gimctl_msg[1].rtr = RT_CAN_DTR;   //数据帧
    gimctl_msg[1].priv = 0;           //报文优先级最高
    gimctl_msg[1].len = 8;            //长度8

    //控制数据清零
    for (int a = 0; a < 8; a++)
    {
        gimctl_msg[0].data[a] = 0;
        gimctl_msg[1].data[a] = 0;
    }
#endif
    SWDG_START(SWDG_GIMBAL_ID);

    while (1)
    {
        /* 等待1ms延时结束 */
        rt_sem_take(&gimbal_1ms_sem, RT_WAITING_FOREVER);

        Frq_Rec(); // 对云台闭环的频率进行检测，通过Jscope相关变量进行记录

        /* 设定值获取 */
        Gimbal_getset(&SetAng);
        // 将大地坐标系下的云台角度设定值/反馈值转化为控制所需的云台坐标系下的角度设定值/反馈值
        GimbalSet_Gound_To_Motor((GimbalCTRL_Set_Type *const *)&SetAng, (AttitudeData_Type *const *)&GroundFrame_GimbalAtti,
                                 (AttitudeData_Type *const *)&MotorFrame_GimbalAtti, &MotorFrame_GimbalErr);

        /* 修改角度设定值 */
        Motor_Write_SetAngle_ABS(&Pitch, SetAng.Pitch);
        Motor_Write_SetAngle_ABS(&Yaw, SetAng.Yaw);

        /* 云台电机控制与电流发送 */
        Gimbal_Controller_Run();

        SWDG_FEED(SWDG_GIMBAL_ID);
    }
}

/**
 * @brief：定时器超时函数，发送信号量，用于控制pid运行周期1ms
 * @param [in]	parameter:该参数不会被使用
 * @return：		无
 * @author：zzj
 */
static void task_1ms_IRQHandler(void *parameter)
{
    while (rt_sem_trytake(&gimbal_1ms_sem) == RT_EOK)
        continue;                    // 清空多余的信号量
    rt_sem_release(&gimbal_1ms_sem); // 重新释放信号量
}

/**
* @brief：初始化云台线程
* @param [in]	无
* @return：		1:初始化成功
                0:初始化失败
* @author：zzj
*/
int gimbal_init(void)
{ //注意电机和IMU初始化需要在这个初始化之前

#if defined CAR_USING_LINK
    Pitch_F0_Use = PTICH_F0_CURRENT;
    Pitch_ANG0_Use = PITCH_ZEROCURRENT_ANG;
#endif

    /* 初始化结构体数据 */
    //初始化电机结构体
    motor_init(&Yaw, YAW_ID, 1, ANGLE_CTRL_ABS, 360, 180, -180, YAW_MOTOR_MIRROR);
    motor_init(&Pitch, PITCH_ID, 1, ANGLE_CTRL_ABS, 360, 180, -180, PITCH_MOTOR_MIRROR);

    while (Yaw.dji.oldangle_state == RT_ERROR || Pitch.dji.oldangle_state == RT_ERROR)
        rt_thread_delay(50);

        /* 初始化PID */
#if (TEST_CLEAR_PID)
    // Yaw
    pid_init(&Yaw.spe, 0, 0, 0, 1000, 15000, -15000); //角速度环
    pid_init(&Yaw.ang, 0, 0, 0, 0.5, 180, -180);      //角度环//绿色步兵：10,0,6//黑色步兵：14,0,4

    // Pitch
    pid_init(&Pitch.spe, 0, 0, 0, 2000, 15000, -15000); //角速度环
    pid_init(&Pitch.ang, 0, 0, 0, 0.5, 100, -100);      //角度环//绿色步兵：10,0,6//黑色步兵：14,0,4
#else
    // Yaw
    pid_init(&Yaw.spe, YAWSPE_PID); //角速度环
    pid_init(&Yaw.ang, YAWANG_PID); //角度环//绿色步兵：10,0,6//黑色步兵：14,0,4
    // Pitch
    pid_init(&Pitch.spe, PITSPE_PID); //角速度环
    pid_init(&Pitch.ang, PITANG_PID); //角度环//绿色步兵：10,0,6//黑色步兵：14,0,4

#endif

    CtrlErr_Calc_Sem = rt_sem_create("CtrlErr", 1, RT_IPC_FLAG_FIFO);

    // 初始化初始位置
    GimbalSet_Init(&SetAng, PITCH_START_ANGLE, gimbal_atti.yaw);

    rt_sem_init(&gimbal_1ms_sem, "GM_Sem", 0, RT_IPC_FLAG_FIFO);

    // 初始化云台PID线程
    gimbal_control = rt_thread_create("GM_CTRL", Gimbal_control_thread, RT_NULL, 4096, THREAD_PRIO_GIMBALPID, 1);
    if (gimbal_control == RT_NULL)
        return RT_ERROR;

    // 创建线程定时器
    rt_timer_init(&task_1ms,
                  "GM_Tim",
                  task_1ms_IRQHandler,
                  RT_NULL,
                  1,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    //线程启动失败返回false
    if (rt_thread_startup(gimbal_control) != RT_EOK)
        return RT_ERROR;

    //启动定时器
    if (rt_timer_start(&task_1ms) != RT_EOK)
        return RT_ERROR;

    return RT_EOK;
}

/**
 * @brief：读取Yaw设定值
 * @param [in]	无
 * @return：		Yaw设定值
 * @author：zzj
 */
float Read_YawSet()
{
    return SetAng.Yaw;
}

/**
 * @brief：读取Pitch设定值
 * @param [in]	无
 * @return：		Pitch设定值
 * @author：zzj
 */
float Read_PitchSet()
{
    return SetAng.Pitch;
}

/**
 * @brief：读取Yaw当前值
 * @param [in]	无
 * @return：		Yaw当前值
 * @author：zzj
 */
float Read_YawNow()
{
    return gimbal_atti.yaw;
}

/**
 * @brief：读取Pitch当前值
 * @param [in]	无
 * @return：		Pitch当前值
 * @author：zzj
 */
float Read_PitchNow()
{
    return gimbal_atti.pitch;
}

/**
 * @brief：读取Yaw角速度当前值
 * @param [in]	无
 * @return：		Yaw角速度当前值
 * @author：zzj
 */
float Read_YawSpeedNow()
{
    return gimbal_atti.yaw_speed;
}

// 传入 0 可停止对云台电机的控制
void Gimbal_Motor_EN(rt_uint8_t GimbalMotor_Enable)
{
    GimbalMotor_Valid = GimbalMotor_Enable;
}
