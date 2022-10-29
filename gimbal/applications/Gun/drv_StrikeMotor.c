#include "drv_StrikeMotor.h"
#include "drv_motor.h"
#include "drv_canthread.h"
#include "mod_Monitor.h"
#include "drv_thread.h"
#include "drv_magazine.h"
#include "drv_utils.h"
#include "robodata.h"

// 本文件包含发射机构相关的电机的控制
#if defined CORE_USING_INFANTRY
#define STRIKEMOTOR_CANDEV can2_dev             // 电机所在CAN设备
static struct rt_semaphore StrikeMotor_2ms_sem; /* 用于接收消息的信号量 */
static struct rt_timer StrikeMotor_Tim;         /* 闭环线程定时器 */
#endif

// 定义电机结构体，闭环状态记录结构体
Motor_t m_rub[2];
Motor_t m_launch = {0};
Motor_CtrlMode_E CTRLMode_Motor[(int)GunMotor_All];

char LaunchMotor_SleepFlag = 0;   // 置 1 则强制设定电机的电流值为 0
char StrikeMotor_Enable_Flag = 1; // 置 0 表示发射机构电机失能
rt_uint8_t Gun_Inited_Flag = 0;   // 用于标志发射机构是否被初始化了

int DeltaRubSpeed; // 摩擦轮转速差

static void (*MotorCTRL_Routine)(void); // 可指定的轮询函数

void StrikeMotor_Enable(int Enable)
{
    StrikeMotor_Enable_Flag = Enable;
}

// 读取发射机构是否被初始化
int Read_Gun_Inited(void)
{
    return (int)Gun_Inited_Flag;
}

// 读取发射机构的指定电机结构体, 注意判断返回值是否为 NULL
Motor_t *Read_Gun_Motor(Gun_Motor_Enum GunMotor)
{
    if (!Gun_Inited_Flag)
        return RT_NULL;
    switch (GunMotor)
    {
    default:
        return RT_NULL;
    case LaunchMotor:
        return &m_launch;
    case RubMotorLeft:
        return &m_rub[0];
    case RubMotorRight:
        return &m_rub[1];
    }
}

// 进行闭环时的编码器数据滞后滤波系数, 该数据为对新数据的置信度
static float Filter_K_Rub = 0.4f;
static float Filter_K_Launch = 0.5f;
// 上次闭环时记录的编码器数据
struct Motor_Encode_Data_s
{
    float Speed; // 编码器角速度
    float Angel; // 编码器角度
};
static struct Motor_Encode_Data_s m_rub_encoder_filtered[2], m_launch_encoder_filtered; // 滤波以后的编码器数据

/**
 * @brief 发射机构电机控制计算函数
 * @author fwlh
 * @param  motor            当前计算的发射机构电机
 * @param  CtrlMode         电机当前的控制模式
 * @param  EncoderData      编码器数据管理结构体
 * @param  Encoder_Filter_K 编码器数据滞后滤波系数, 对旧数据的置信程度
 * @return float            当前电机需要输出的控制数据
 */
static float StrikeMotor_Ctrl_Calc(Motor_t* motor, Motor_CtrlMode_E CtrlMode, struct Motor_Encode_Data_s *EncoderData, float Encoder_Filter_K)
{
    // 发射机构未初始化的时候返回 0
    if (!Read_Gun_Inited())
        return 0.f;
    // 传入结构体指针为空时返回 0
    if ((!motor) || (!EncoderData))
        return 0.f;
    switch (CtrlMode)
    {
    default:
    case MOTORCTRL_CLR:
        return 0.f;
    case MOTORCTRL_SPE:
        EncoderData->Speed = UTILS_LP_FAST(EncoderData->Speed, Motor_Read_NowSpeed(motor), Encoder_Filter_K);
        return Motor_SpeedPIDCalculate(motor, EncoderData->Speed);
    case MOTORCTRL_ANG:
        EncoderData->Angel = utils_circle_number_LowPass_Filter(EncoderData->Angel, Motor_Read_NowAngle(motor), Encoder_Filter_K, 0, 8192);
        Motor_AnglePIDCalculate(motor, EncoderData->Angel);
        Motor_Write_SetSpeed_FromAnglePID(motor);
        EncoderData->Speed = UTILS_LP_FAST(EncoderData->Speed, Motor_Read_NowSpeed(motor), Encoder_Filter_K);
        return Motor_SpeedPIDCalculate(motor, EncoderData->Speed);
    }
}

// 定时运行的发射机构控制函数
void StrikeMotor_CtrlRoutine(StrikeMotor_CtrlData_s *DataOut)
{
    // 未初始化时所有数据输出 0
    if (!Gun_Inited_Flag)
    {
        DataOut->DataOut[(int)LaunchMotor] = 0.f;
        DataOut->DataOut[(int)RubMotorRight] = 0.f;
        DataOut->DataOut[(int)RubMotorLeft] = 0.f;
        return;
    }
    // 判断发射机构电机是否使能
    if (StrikeMotor_Enable_Flag)
    {
        if (MotorCTRL_Routine != NULL)
            MotorCTRL_Routine(); // 执行指定的轮询函数

        if (DataOut == RT_NULL)
            return;

        DataOut->DataOut[(int)RubMotorLeft] = StrikeMotor_Ctrl_Calc(Read_Gun_Motor(RubMotorLeft), CTRLMode_Motor[(int)RubMotorLeft], &m_rub_encoder_filtered[0], Filter_K_Rub);
        DataOut->DataOut[(int)RubMotorRight] = StrikeMotor_Ctrl_Calc(Read_Gun_Motor(RubMotorRight), CTRLMode_Motor[(int)RubMotorRight], &m_rub_encoder_filtered[1], Filter_K_Rub);
        DataOut->DataOut[(int)LaunchMotor] = StrikeMotor_Ctrl_Calc(Read_Gun_Motor(LaunchMotor), CTRLMode_Motor[(int)LaunchMotor], &m_launch_encoder_filtered, Filter_K_Launch);

        // 判断现在是不是需要播弹盘电机休息
        if (LaunchMotor_SleepFlag)
            DataOut->DataOut[(int)LaunchMotor] = 0.f;

        // 记录左右摩擦轮的转速差
        DeltaRubSpeed = abs(abs(m_rub[0].dji.speed) - abs(m_rub[1].dji.speed));
    }
    else
    {
        DataOut->DataOut[(int)LaunchMotor] = 0.f;
        DataOut->DataOut[(int)RubMotorRight] = 0.f;
        DataOut->DataOut[(int)RubMotorLeft] = 0.f;
    }
}

#if defined CORE_USING_INFANTRY
StrikeMotor_CtrlData_s StrikeMotor_CtrlData; // 计算发射机构电机 PID 计算的结果
/* StrikeMotor_Tim 超时函数 */
static void StrikeMotor_2ms_IRQHandler(void *parameter)
{
    while (rt_sem_trytake(&StrikeMotor_2ms_sem) == RT_EOK)
        continue; // 清空多余的信号量
    rt_sem_release(&StrikeMotor_2ms_sem);
}

/**
 * @brief 电机通信入口函数
 * @author fwlh
 * @param  parameter        没有被使用
 */
static void StrikeMotor_2ms_entry(void *parameter)
{
    struct rt_can_msg txmsg;
    txmsg.id = STDID_launch;
    txmsg.ide = RT_CAN_STDID;
    txmsg.rtr = RT_CAN_DTR;
    txmsg.len = 8;

    SWDG_START(SWDG_STRIKE_ID);
    while (1)
    {
        // 发射机构控制相关计算
        StrikeMotor_CtrlRoutine(&StrikeMotor_CtrlData);
        // 电机通信
        txmsg.data[(int16_t)(LAUNCH_ID - 0x201) * 2] = (rt_uint8_t)((rt_int16_t)StrikeMotor_CtrlData.DataOut[(int)LaunchMotor] >> 8);
        txmsg.data[(int16_t)(LAUNCH_ID - 0x201) * 2 + 1] = (rt_uint8_t)((rt_int16_t)StrikeMotor_CtrlData.DataOut[(int)LaunchMotor]);
        txmsg.data[(int16_t)(ID_RUB_LEFT - 0x201) * 2] = (rt_uint8_t)((rt_int16_t)StrikeMotor_CtrlData.DataOut[(int)RubMotorLeft] >> 8);
        txmsg.data[(int16_t)(ID_RUB_LEFT - 0x201) * 2 + 1] = (rt_uint8_t)((rt_int16_t)StrikeMotor_CtrlData.DataOut[(int)RubMotorLeft]);
        txmsg.data[(int16_t)(ID_RUB_RIGHT - 0x201) * 2] = (rt_uint8_t)((rt_int16_t)StrikeMotor_CtrlData.DataOut[(int)RubMotorRight] >> 8);
        txmsg.data[(int16_t)(ID_RUB_RIGHT - 0x201) * 2 + 1] = (rt_uint8_t)((rt_int16_t)StrikeMotor_CtrlData.DataOut[(int)RubMotorRight]);
        rt_device_write(STRIKEMOTOR_CANDEV, 0, &txmsg, sizeof(txmsg));
        // 等待下一个定时周期
        rt_sem_take(&StrikeMotor_2ms_sem, RT_WAITING_FOREVER);
        SWDG_FEED(SWDG_STRIKE_ID);
    }
}
#endif

static rt_int16_t GunSpeedNow = 0; // 记录当前摩擦轮转速设定值
// rt_int16_t setspeed;
/**
 * @brief  摩擦轮转速设定
 * @param  speed：转速
 */
void Rub_speed_set(rt_int16_t speed)
{
    if (GunSpeedNow != speed)
    {
        GunSpeedNow = speed;
#ifdef RUB_SNAIL
        motor_rub_set(GunSpeedNow);
#else

        if (speed == 0)
        { // 如果转速设定值为0，则开环降速
            CTRLMode_Motor[(int)RubMotorLeft] = MOTORCTRL_CLR;
            CTRLMode_Motor[(int)RubMotorRight] = MOTORCTRL_CLR;
            pid_clear(&m_rub[0].spe);
            pid_clear(&m_rub[1].spe);
        }
        else
        {
            CTRLMode_Motor[(int)RubMotorLeft] = MOTORCTRL_SPE;
            CTRLMode_Motor[(int)RubMotorRight] = MOTORCTRL_SPE;
            Motor_Write_SetSpeed_ABS(&m_rub[1], -GunSpeedNow);
            Motor_Write_SetSpeed_ABS(&m_rub[0], GunSpeedNow);
        }
#endif
    }
}

// 读取当前摩擦轮转速设定值
rt_int16_t Rub_speed_ReadSet(void)
{
    if (GunSpeedNow != -1)
        return GunSpeedNow;
    else
        return 0;
}

// 读取当前摩擦轮是否开启
rt_bool_t Read_Rub_Started(void)
{
    if ((abs(m_rub[0].dji.speed) > 200) && (abs(m_rub[1].dji.speed) > 200))
        return ((StrikeMotor_Enable_Flag && Rub_speed_ReadSet()) ? RT_TRUE : RT_FALSE);
    else
        return RT_FALSE;
}

#if defined CORE_USING_INFANTRY
/**
 * @brief  任务创建
 */
static void strike_start(void)
{
    /*定时器处理线程*/
    rt_thread_t thread;
    rt_sem_init(&StrikeMotor_2ms_sem, "StrM_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("StrMCtrl", StrikeMotor_2ms_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器中断*/
    rt_timer_init(&StrikeMotor_Tim, "Strk_Tim", StrikeMotor_2ms_IRQHandler, RT_NULL, 2,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 启动定时器 */
    rt_timer_start(&StrikeMotor_Tim);
}
#endif

static void WaitForMotorData(void)
{
    char WaitCount = 0;
    m_launch.dji.FreshTick = 0;
    while (1)
    {
        if (m_launch.dji.FreshTick != 0)
        {
            rt_thread_delay(10);
            return;
        }
        else
        {
            rt_thread_delay(10);
            WaitCount++;
            if (WaitCount > 80)
                return;
        }
    }
}

/**
 * @brief  发射机构电机闭环初始化
 */
void StrikeMotor_init(void)
{
    MotorCTRL_Routine = NULL; // 默认没有轮询函数
    //电机初始化
#ifdef RUB_SNAIL
    motor_rub_init();
#else //默认3508
    motor_init(&m_rub[0], ID_RUB_LEFT, 1, ANGLE_CTRL_EXTRA, 8192, 8192, 0, GUN_RUB_TOGGLE);
    motor_init(&m_rub[1], ID_RUB_RIGHT, 1, ANGLE_CTRL_EXTRA, 8192, 8192, 0, GUN_RUB_TOGGLE);
#endif
#if defined CORE_USING_INFANTRY
    // 初始化拨弹电机
    motor_init(&m_launch, LAUNCH_ID, 36.f, ANGLE_CTRL_FULL, 8192, 360, 0, GUN_LAUNCH_TOGGLE);
#elif defined CORE_USING_HERO
    // 初始化拨弹电机
    motor_init(&m_launch, LAUNCH_ID, 19.202f, ANGLE_CTRL_FULL, 8192, 360, 0, GUN_LAUNCH_TOGGLE);
#endif
    WaitForMotorData();
#if defined CORE_USING_INFANTRY
    //舵机初始化
    Magazine_servo_init();
#endif
    // PID初始化
#if TEST_CLEAR_PID
    pid_init(&m_launch.ang, PID_CLEAR);
    pid_init(&m_launch.spe, PID_CLEAR);
    pid_init(&m_rub[0].spe, PID_CLEAR);
    pid_init(&m_rub[1].spe, PID_CLEAR);
#else
    pid_init(&m_launch.ang, LAUNCH_ANG_PID);
    pid_init(&m_launch.spe, LAUNCH_SPE_PID);
    pid_init(&m_rub[0].spe, RUB_SPE_PID);
    pid_init(&m_rub[1].spe, RUB_SPE_PID);
#endif
    //拨弹电机初值为0（这里的设定值以电机上电时刻角度为零位）
    Motor_Write_SetAngle_ABS(&m_launch, 0);

    // 设置默认闭环状态 摩擦轮不闭环，拨弹角度闭环
    CTRLMode_Motor[(int)RubMotorLeft] = MOTORCTRL_CLR;
    CTRLMode_Motor[(int)RubMotorRight] = MOTORCTRL_CLR;
    CTRLMode_Motor[(int)LaunchMotor] = MOTORCTRL_ANG;
#if defined CORE_USING_INFANTRY
    //创建发射机构线程
    strike_start();
#endif
    Gun_Inited_Flag = 1;
}

// 指定轮询函数指针
void CTRLRoutine_Set(void (*Func)(void))
{
    MotorCTRL_Routine = Func;
}
