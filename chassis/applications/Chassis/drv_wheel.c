#include "drv_wheel.h"
#include "drv_motor.h"
#include "drv_canthread.h"
#include "HCanID_data.h"
#include "HThread_data.h"
#include "HChassis_Data.h"
#include "SuperCap_Com.h"
#include "drv_utils.h"
#include "mod_Monitor.h"
#include "app_GetGim.h"

#ifdef MOTOR_USE_SYNC
#include "drv_MotorSync.h"
#endif

#ifdef MOTOR_USE_SYNC
MotorSyncSettings_S MotorSync_Settings = {0};
MotorSync_Data_S MotorSync_Data = {0};

int Wheels_Sync_Init(void)
{
#if defined CORE_USING_INFANTRY
    MotorSync_Settings.Input_Gain = 1.f;
#elif defined CORE_USING_HERO
    MotorSync_Settings.Input_Gain = 0.5f;
#endif
    MotorSync_Settings.LowSpeed_Set = 100;
    MotorSync_Settings.LossMax_Set = 50000;
    MotorSync_Settings.ADJ_K = 0.005f;
    MotorSync_Settings.OverFlow_K = 1.5f;
    MotorSync_Settings.IDat_Sign = 1;
    MotorSync_Settings.Motor_Count = (int)WHEELS_NUM;
    MotorSync_Settings.Alpha_Max = 1.5f;
    MotorSync_Settings.Alpha_Min = 0.1f;
    MotorSync_Settings.Break_K = 1;
    MotorSync_Settings.Alpha_Filter_K = 0.7f;
    MotorSync_Settings.Speed_Filter_K = 0.8f;
    MotorSync_Settings.Recover_K = 0.13f;
    MotorSync_Settings.IData_damp = 0.25f;
    MotorSync_Settings.IData_Max = 70000;
    MotorSync_Settings.IData_Min = -70000;

    MotorSyncCTRL_Init(&MotorSync_Settings, &MotorSync_Data);
    return 1;
}
#endif

Motor_t wheel[WHEELS_NUM];
rt_uint8_t Wheel_Enabled = 1; // 写 0 可以将底盘电机失能

// 使能/失能底盘电机
void Wheel_Enable(int EN)
{
    Wheel_Enabled = (rt_uint8_t)EN;
}

/*车轮控制模式*/
static Wheels_manage_t wheels_ctrl = {
    .ctrl_mode = LOOP_CTRL,
    .bound_TL_Low = LOW_BATTERY_TL,
    .bound_TH_Low = LOW_BATTERY_TH,
    .bound_TL_High = RESERVE_BATTERY_TL,
    .bound_TH_High = RESERVE_BATTERY_TH,
    .if_SC_protect = RT_TRUE,
    .if_ReserveEnergy = RT_TRUE};

/*线程定时器*/
static struct rt_timer Cal_Timer;
static struct rt_semaphore Cal_sem;
static void Cal_IRQHandler(void *parameter)
{
    rt_sem_release(&Cal_sem);
}

/**
 * @brief   限制轮子的输入设定速度
 * @param   max_limit 最大速度，单位rpm
 * @return  None
 */
static void Wheels_Speed_Limit(float max_limit)
{
    float set_max, rate;

    /*限制输入为正数*/
    max_limit = fabsf(max_limit);

    /*找到4个值的绝对值的最大值*/
    set_max = utils_max_of_4(fabsf(Wheel_Read_SetSpeed(WHEEL_RF)),
                             fabsf(Wheel_Read_SetSpeed(WHEEL_LF)),
                             fabsf(Wheel_Read_SetSpeed(WHEEL_LB)),
                             fabsf(Wheel_Read_SetSpeed(WHEEL_RB)));

    if (set_max > max_limit)
    {
        /*同比例缩小速度设定值*/
        rate = max_limit / set_max;
        for (int lo = 0; lo < WHEELS_NUM; lo++)
            Motor_Write_SetSpeed_ABS(&wheel[lo], rate * Wheel_Read_SetSpeed((Wheel_local_e)lo));
    }
}

#ifdef USE_CAPACITY
/**
 * @brief   限制设定的总电流大小
 * @param   max_limit 最大电流，单位与所选择电机有关
 * @return  None
 */
static void Wheels_Current_Limit(float max_limit)
{
    float Sum_I = 0, rate;

    /*限制输入为正数*/
    max_limit = fabsf(max_limit);

    /*计算当前设定的总输出电流大小*/
    for (int lo = 0; lo < WHEELS_NUM; lo++)
        Sum_I += fabsf(wheel[lo].spe.out);

    /*当总输出电流太大*/
    if (Sum_I > max_limit)
    {
        //计算削减比例
        rate = max_limit / Sum_I;
        for (int lo = 0; lo < WHEELS_NUM; lo++)
            wheel[lo].spe.out *= rate;
    }
}

static float Current_Attenuation_Rate = 0.f; // 设定电流衰减比例
/**
 * @brief   超级电容剩余电量保护
 * @param   whectrl Wheels_manage_t 指针
 * @return  None
 */
static void Wheels_SC_Protect(Wheels_manage_t *whectrl)
{
    float Sc = Get_RemainCapcity();
    whectrl->if_ReserveEnergy = (bool)Get_PowerRestriction_Status();

    // 判断是不是需要做超级电容电量预留
    if (whectrl->if_ReserveEnergy)
    {
        // 需要做电量预留
        if (Sc >= whectrl->bound_TH_High)
        {
            Current_Attenuation_Rate = 1.f;
            whectrl->if_hysteresis = RT_FALSE;
        }
        else if ((Sc < whectrl->bound_TH_High) && (Sc >= whectrl->bound_TL_High))
        {
            whectrl->if_hysteresis = RT_TRUE;
            Current_Attenuation_Rate = 1 - SQUARE(1 - utils_calc_ratio(whectrl->bound_TL_High, whectrl->bound_TH_High, Sc));
            utils_truncate_number(&Current_Attenuation_Rate, 0.2f, 1.f);
        }
        else if ((Sc < whectrl->bound_TL_High) && (Sc >= whectrl->bound_TH_Low))
        {
            whectrl->if_hysteresis = RT_TRUE;
            Current_Attenuation_Rate = 0.2f;
        }
        else if ((Sc <= whectrl->bound_TH_Low) && (Sc > whectrl->bound_TL_Low))
        {
            whectrl->if_hysteresis = RT_TRUE;
            Current_Attenuation_Rate = 0.2f * utils_calc_ratio(whectrl->bound_TL_Low, whectrl->bound_TH_Low, Sc);
        }
        else if (Sc <= whectrl->bound_TH_Low)
        {
            Current_Attenuation_Rate = 0.f;
            whectrl->if_hysteresis = RT_TRUE;
        }
    }
    else
    {
        // 不需要做电量预留
        if (Sc > whectrl->bound_TH_Low)
        {
            Current_Attenuation_Rate = 1.f;
            whectrl->if_hysteresis = RT_FALSE;
        }
        // 一段滞回的空间，充到15%以上再正常工作
        else if ((Sc <= whectrl->bound_TH_Low) && (Sc > whectrl->bound_TL_Low))
        {
            //超级电容没电，削减电流输出, 接近没电的状态下快速降低设定值
            Current_Attenuation_Rate = SQUARE(utils_calc_ratio(whectrl->bound_TL_Low, whectrl->bound_TH_Low, Sc));
            whectrl->if_hysteresis = RT_TRUE;
        }
        else if (Sc <= whectrl->bound_TH_Low)
        {
            Current_Attenuation_Rate = 0.f;
            whectrl->if_hysteresis = RT_TRUE;
        }
    }
    Wheels_Current_Limit(Current_Attenuation_Rate * CURRENT_LIMIT);
}
#endif /* USE_CAPACITY */

static uint8_t Wheels_Motor_Offline = 0; // 用于记录底盘电机离线情况, 从低到高分别为：右前、左前、左后、右后
/**
 * @brief   车轮pid计算线程
 * @param   parameter
 * @return  None
 */
static void PID_Cal_Thread(void *parameter)
{
    static float alpha;
#ifdef MOTOR_USE_SYNC
    MotorInfo_Input_S SpeedState;
    float NewSet[4];
#endif
    struct rt_can_msg txmsg; // 发送电流报文结构体
    txmsg.id = STDID_launch;
    txmsg.ide = RT_CAN_STDID;
    txmsg.rtr = RT_CAN_DTR;
    txmsg.len = 8;
    SWDG_START(SWDG_WHEELS_PID_ID); // 开启底盘 PID 线程监视器
    while (1)
    {
        rt_sem_take(&Cal_sem, RT_WAITING_FOREVER);

        if ((wheels_ctrl.ctrl_mode == LOOP_CTRL) && (Wheel_Enabled))
        {
            //输入限制
            Wheels_Speed_Limit(RATED_SPEED);

#ifdef MOTOR_USE_SYNC
            if (!Wheels_Motor_Offline)
                //速度闭环
                alpha = Sync_Alpha_Cal(&MotorSync_Settings, &MotorSync_Data);
            else
                alpha = 1.f;
#else
            alpha = 1.f;
#endif
            for (int lo = 0; lo < WHEELS_NUM; lo++)
            {
#ifdef MOTOR_USE_SYNC
                NewSet[lo] = Motor_Read_SetSpeed(&wheel[lo]);
#endif
                Motor_Write_SetSpeed_ABS(&wheel[lo], alpha * Motor_Read_SetSpeed(&wheel[lo]));
                float SpeedNow = Motor_Read_NowSpeed(&wheel[lo]); //控制输入单位:快转子的rpm;
                if ((wheel[lo].dji.FreshTick) && (rt_tick_get() - wheel[lo].dji.FreshTick < 200))
                {
                    Motor_SpeedPIDCalculate(&wheel[lo], SpeedNow); // pid计算结果,单位:电流
                    Wheels_Motor_Offline &= ~(1 << lo);
                }
                else
                {
                    wheel[lo].spe.out = 0;
                    Wheels_Motor_Offline |= (1 << lo);
                }
            }

#ifdef MOTOR_USE_SYNC
            if (!Wheels_Motor_Offline)
            {
                //底盘同步计算
                for (int i = 0; i < (int)WHEELS_NUM; ++i)
                {
                    SpeedState.SpeedSenseNow = Motor_Read_NowSpeed(&wheel[i]);
                    SpeedState.SpeedSetNow = Motor_Read_SetSpeed(&wheel[i]);
                    SpeedState.SpeedSet_BeforeAlpha = NewSet[i];
                    MotorSync_MotorInput(&SpeedState, &MotorSync_Data, i);
                }
            }
#endif
#ifdef USE_CAPACITY
            //低电量保护
            if (wheels_ctrl.if_SC_protect == RT_TRUE)
                Wheels_SC_Protect(&wheels_ctrl);
#endif /* USE_CAPACITY */

            //发送电流报文
            for (int i = 0; i < (int)WHEELS_NUM; ++i)
            {
                txmsg.data[(wheel[i].dji.motorID - 0x201) * 2] = (rt_uint8_t)(((rt_int16_t)Motor_Read_OutSpeed(&wheel[i])) >> 8);
                txmsg.data[(wheel[i].dji.motorID - 0x201) * 2 + 1] = (rt_uint8_t)(((rt_int16_t)Motor_Read_OutSpeed(&wheel[i])));
            }
            rt_device_write(can1_dev, 0, &txmsg, sizeof(txmsg));
        }
        else
            motor_current_send(can1_dev, STDID_launch, 0, 0, 0, 0);
        SWDG_FEED(SWDG_WHEELS_PID_ID);
    } // while
}

/**
 * @brief   车轮电机初始化
 *          并且等待can1接收到电机报文
 * @param   None
 * @return  None
 */
static void Wheels_Motors_Init(void)
{
    /*初始化电机结构体*/
    motor_init(&wheel[WHEEL_RF], RIGHT_FRONT, REDUCT_RATIO, ANGLE_CTRL_EXTRA, 8192, 180, -180);
    motor_init(&wheel[WHEEL_LF], LEFT_FRONT, REDUCT_RATIO, ANGLE_CTRL_EXTRA, 8192, 180, -180);
    motor_init(&wheel[WHEEL_LB], LEFT_BACK, REDUCT_RATIO, ANGLE_CTRL_EXTRA, 8192, 180, -180);
    motor_init(&wheel[WHEEL_RB], RIGHT_BACK, REDUCT_RATIO, ANGLE_CTRL_EXTRA, 8192, 180, -180);

    /*pid初始化*/
    pid_init(&wheel[WHEEL_RF].spe, SPE_PID_PARAMETER_RF);
    pid_init(&wheel[WHEEL_LF].spe, SPE_PID_PARAMETER_LF);
    pid_init(&wheel[WHEEL_LB].spe, SPE_PID_PARAMETER_LB);
    pid_init(&wheel[WHEEL_RB].spe, SPE_PID_PARAMETER_RB);

    /*等待电机第一次通信完毕*/
    rt_uint8_t CheckSum = 0, CheckTimes = 0;
    while (RT_TRUE)
    {
        CheckSum = 0;
        for (int lo = 0; lo < (int)WHEELS_NUM; ++lo)
            CheckSum += wheel[lo].dji.Data_Valid;
        if (CheckSum == (int)WHEELS_NUM)
            break;
        // 如果只有一个轮子离线也是勉强可以开始控制的
        else if (CheckSum == (int)WHEELS_NUM - 1)
        {
            ++CheckTimes;
            if (CheckTimes > 20)
                break;
        }
        rt_thread_mdelay(50);
    }
}

/**
 * @brief   车轮初始化
 * @param   None
 * @return  None
 * @author  lfp
 */
rt_err_t Wheels_Init(void)
{
    rt_err_t res;
    rt_thread_t thread = RT_NULL;

    //初始化电机结构体
    Wheels_Motors_Init();

#ifdef MOTOR_USE_SYNC
    // 初始化底盘同步
    Wheels_Sync_Init();
#endif

    //初始化信号量
    res = rt_sem_init(&Cal_sem, "Cal_sem", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;

    //初始化底盘线程
    thread = rt_thread_create("PID_Cal_Thread",        //线程名
                              PID_Cal_Thread,          //线程入口
                              RT_NULL,                 //入口参数无
                              THREAD_STACK_WHEELS_CAL, //线程栈
                              THREAD_PRIO_WHEELS_CAL,  //线程优先级
                              THREAD_TICK_WHEELS_CAL); //线程时间片大小

    //线程创建失败返回false
    if (thread == RT_NULL)
        return RT_ERROR;

    //线程启动失败返回false
    if (rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;

    //创建线程定时器
    rt_timer_init(&Cal_Timer,
                  "Cal_Timer",
                  Cal_IRQHandler,
                  RT_NULL,
                  W_CAL_PERIOD,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    //启动定时器
    res = rt_timer_start(&Cal_Timer);
    if (res != RT_EOK)
        return res;

    return RT_EOK;
}

///////////////////////////////////////对外接口/////////////////////////////////////////
/**
 * @brief   读取驱动电机can报文
 * @param   msg 反馈报文
 * @param   local 车轮位置
 */
void Refresh_Wheels_Motor(struct rt_can_msg *msg, Wheel_local_e local)
{
    motor_readmsg(msg, &wheel[local].dji);
}

/**
 * @brief   设定轮子的线速度大小
 * @param   local 车轮位置
 * @param   speed 设定速度，>0轮子带着底盘往前转(全向轮：0,2轮带底盘前走；1,3轮带底盘右走)，单位mm/s
 */
void Wheel_Speed_Set(Wheel_local_e local, float speed)
{
    /*物理量和单位转化为快转子的rpm*/
    speed = speed / (float)WHEEL_RADIUS * (30.0f / PI) * REDUCT_RATIO; // WHEEL_RADIUS*(30.0/USER_PI)为单位转换的系数，右式speed为期望的轮子线速度，单位：rpm，REDUCTION_RATIO为减速比，被控目标为大转子的角速度rpm

    /*根据电机安装朝向，修正速度方向*/
    if (INSTALL_DIR == W_INSIDE)
    {
        speed = -speed;
    }

    /*根据电机安装位置，修正速度方向*/
    switch (local)
    {
    case WHEEL_RF:
    case WHEEL_RB:
        Motor_Write_SetSpeed_ABS(&wheel[local], -speed);
        break;

    case WHEEL_LF:
    case WHEEL_LB:
        Motor_Write_SetSpeed_ABS(&wheel[local], speed);
        break;

    default:
        RT_ASSERT(0);
        break;
    }
}

/**
 * @brief   改变车轮的控制模式
 * @param   mode  闭环还是不闭环
 * @return  None
 */
void Wheels_Change_Mode(Crtl_e mode)
{
    wheels_ctrl.ctrl_mode = mode;

    /*默认使用无闭环模式后，底盘设定速度降为0*/
    if (mode == NO_CTRL)
    {
        for (int lo = 0; lo < WHEELS_NUM; lo++)
            Wheel_Speed_Set((Wheel_local_e)lo, 0);
    }
}

/**
 * @brief   设置是否启用超级电容保护模式
 * @param   if_Sc  RT_TRUE,启用；RT_FALSE，关闭
 * @return  None
 */
void Wheels_If_ScProtect(rt_bool_t if_Sc)
{
    wheels_ctrl.if_SC_protect = if_Sc;
}

/**
 * @brief   获取当前是否在低电量保护
 * @param   None
 * @return  rt_bool_t RT_TRUE,保护；RT_FALSE，未保护
 */
rt_bool_t Wheels_Read_ScProtectState(void)
{
    return wheels_ctrl.if_hysteresis;
}

/**
 * @brief    修改底盘四个电机的pid参数,暂时用在调试上
 * @param    kp-ki-kd    pid参数
 * @return   None
 */
void Wheels_Modify_Spid(float kp, float ki, float kd, float i_limit, float out_limit_up, float out_limit_down)
{
    for (int lo = 0; lo < WHEELS_NUM; lo++)
        pid_init(&wheel[lo].spe, kp, ki, kd, i_limit, out_limit_up, out_limit_down);
}

/**
 * @brief   读取轮子的速度设定值
 * @param   local 车轮位置
 */
float Wheel_Read_SetSpeed(Wheel_local_e local)
{
    return Motor_Read_SetSpeed(&wheel[local]);
}

/**
 * @brief   读取轮子的当前速度(输出麦轮转速 mm/s)
 * @param   local 车轮位置
 */
float Wheel_Read_NowSpeed(Wheel_local_e local)
{
    /*物理量和单位转化为快转子的rpm*/
    float speed = Motor_Read_NowSpeed(&wheel[local]) / REDUCT_RATIO / 30 * PI * WHEEL_RADIUS;

    /*根据电机安装朝向，修正速度方向*/
    if (INSTALL_DIR == W_INSIDE)
    {
        speed = -speed;
    }

    /*根据电机安装位置，修正速度方向*/
    switch (local)
    {
    case WHEEL_RF:
    case WHEEL_RB:
        return -speed;

    case WHEEL_LF:
    case WHEEL_LB:
        return speed;

    default:
        return 0.f;
    }
}

/**
 * @brief 读取电机的电流设定值
 * @author fwlh
 * @param  local            车轮位置
 * @return float            设定电流
 */
float Wheel_Read_SetCurrent(Wheel_local_e local)
{
    return wheel[local].spe.out;
}

/**
 * @brief 读取电机的转速 PID 结构体
 * @author fwlh
 * @param  local            车轮位置
 * @return pid_t*           转速 PID 结构体
 */
pid_t *Wheel_Read_PID(Wheel_local_e local)
{
    return &wheel[local].spe;
}

/**
 * @brief 读取底盘电机当前的离线情况
 * @author fwlh
 * @return rt_uint8_t   从低到高分别为：左前、左后、右后、右前, 对应位置 1 代表离线
 */
rt_uint8_t Read_Wheels_Offline_State(void)
{
    return Wheels_Motor_Offline & ((1 << 4) - 1);
}
