#include "mod_charge.h"
#include "mod_can.h" // 看门狗需要在这里检测CAN的状态
#include "drv_monitor.h"
#include "app_charge.h"

static rt_thread_t Charge_Ctrl_thread_tid = RT_NULL; // 功率闭环控制线程句柄

static rt_timer_t Charge_Ctrl_Timer;              // 用于控制充电算法执行周期的定时器
static rt_sem_t Charge_Ctrl_Timer_dsem = RT_NULL; // 定时器回调函数中用于同步算法计算线程的信号量

float P_set = 0; // 底盘功率设定值

static float P_Fix_D_set = P_FIX_D_SET_DEF;

static float P_fix; // 充电输出功率补偿计算值

float P_app;             // 充电模块输出功率实际应用值
float P_IN_Now;          // 底盘功率实际计算值
static float P_err;      // 底盘功率控制时的偏差量
static float P_err_last; //上次的底盘功率偏差量

struct Power_State_s
{
    float PCE;            // 当前的电源效率
    float P_Real;         // 本次估计出来的下周期的实际功率
    rt_bool_t Flag_Short; // 发生短路
    rt_bool_t Flag_Full;  // 充满了
} Power_State = {.P_Real = 0.f, .Flag_Short = 0, .Flag_Full = 0};

//读取当前充电功率设定值
rt_uint16_t Get_Charge_PowerSet()
{
    return (rt_uint16_t)P_set;
}

// 修改功率设定值
void Charge_Ctrl_set_P(float P_Set_New)
{
    P_set = P_Set_New;
}

// 充电控制算法-周期控制定时器-超时函数
void Charge_Ctrl_Timer_TimeOut(void *para)
{
    rt_sem_release(Charge_Ctrl_Timer_dsem); //定时器触发, 释放信号量
}

/**
 * @brief 通过当前的设定功率计算出本周期结束后模块的状态
 * @author fwlh
 * @param  P_Set            本次的设定功率
 * @param  P_IN             当前的输入功率
 * @param  Power_State      模块状态
 */
void Power_Est(float P_Set, float P_IN, struct Power_State_s *Power_State)
{
    float temp_P_Real = P_Set * POWER_EST_FILTER_SET + Power_State->P_Real * (1 - POWER_EST_FILTER_SET);
//     float delta_P = temp_P_Real - P_IN;

    float temp_PCE = temp_P_Real / P_IN_Now; // 计算出电源效率
    if (temp_PCE > PCE_CAL_MAX)
    {
        temp_PCE = PCE_CAL_MAX;            // 电源效率计算值如果出现很大误差，则需进行修正
        temp_P_Real = temp_PCE * P_IN_Now; // 根据新的电源效率估计实际功率
        Power_State->Flag_Full = 1;
        Power_State->Flag_Short = 0;
    }
    else if (temp_PCE < PCE_CAL_MIN)
    {
        temp_PCE = PCE_CAL_MIN; // 电源效率计算值如果出现很大误差，则需进行修正
        temp_P_Real = temp_PCE * P_IN_Now;
        Power_State->Flag_Short = 1;
        Power_State->Flag_Full = 0;
    }
    else
    {
        Power_State->Flag_Short = 0;
        Power_State->Flag_Full = 0;
    }

    Power_State->PCE = temp_PCE;
    Power_State->P_Real = temp_P_Real;
}

// 按照当前的电容电压设置充电模块输出端的输出功率，若电容电压低于4V，则计算时按照4V计算
float Charge_Set_CHGOUT_P(float P_CHGOUT_Set)
{
    float V_CAP_USE; //用于限幅
    float I_Set;
    V_CAP_USE = adc_data.UseDat.V_CAP;
    if (V_CAP_USE < 2)
    {
        V_CAP_USE = 2;
    }
    I_Set = P_CHGOUT_Set / V_CAP_USE;
    I_Set = CHG_Set_Curr(I_Set);
    return adc_data.UseDat.V_CAP * I_Set;
}

// 除数0修正
#define Zero_fix(dat, min) _Zero_fix(&dat, min) // 简化取地址符
void _Zero_fix(float *dat, float min)
{
    if (*dat < min)
    {
        *dat = min;
    }
}

void toggle_LED2(void)
{
    static int rem = 0;
    rem++;
    if (rem == 15)
    {
        HWFUN_LED2_ON;
    }
    else if (rem == 45)
    {
        rem = 0;
        HWFUN_LED2_OFF;
    }
}

// 周期性运行的充电控制算法函数
void Charge_Ctrl_Thread(void *para)
{
    float P_err_D;
    P_err = adc_data.UseDat.V_IN * adc_data.UseDat.I_IN - P_set; // 首次计算, P_err直接取当前值
    char first_flag = 1;

#if (TEST_LIMIT_CAP_VOLTAGE)
    char Full_Flag = 0;
#endif

    while (1)
    {
        // 等待软件定时器发送信号量
        rt_sem_take(Charge_Ctrl_Timer_dsem, RT_WAITING_FOREVER);

        // 使用func_adc函数计算ADC标定数值
        adc_DataProcess();

        toggle_LED2(); // 充电指示灯闪烁

        //控制底盘电源
        if (adc_data.UseDat.V_CAP > CAP_VOLTAGE_EPT + 2.0f)
        {
            SPLY_Set_Curr(14.0);
            HWFUN_SPLY_EN_PIN_EN;
            HWFUN_LED3_ON;
        }
        if (adc_data.UseDat.V_CAP < CAP_VOLTAGE_EPT)
        {
            if (adc_data.UseDat.V_CAP < CAP_VOLTAGE_EPT - 1.0f)
            { // 电量严重不足，直接关闭输出
                SPLY_Set_Curr(0.0);
                HWFUN_SPLY_EN_PIN_DIS;
                HWFUN_LED3_OFF;
            }
            else
            {
                SPLY_Set_Curr(1.5f); // 电量余量不多，进行限流
            }
        }

#if (WDT_AlwaysFeed == 1)
        WDT_Feed(); // 直接喂狗
#else
        // 看门狗相关
        if (ADC_Fresh == 1 && MOD_CAN_Fresh == 1)
        { // adc运行正常，继续检查其它条件
            ADC_Fresh = 0;
            MOD_CAN_Fresh = 0;
            if (adc_data.UseDat.V_CAP < CAP_VOLTAGE_FULL - 2)
            { // 超级电容电量不满
                if (adc_data.UseDat.I_IN > 0.3f)
                { // 正常充电，可以喂狗
                    WDT_Feed();
                }
            }
            else
            {               // 超级电容电量较满
                WDT_Feed(); // 直接喂狗
            }
        }
        // 收到不充电的指令且充电电流比较小的时候也可以喂狗
        if ((Get_Charge_Permission()) && (Get_CAN_Return_Valid() == RT_EOK) && (adc_data.UseDat.I_IN < 0.5f))
            WDT_Feed(); // 直接喂狗
#endif

#if (TEST_LIMIT_CAP_VOLTAGE)
        // 满电停止充电（可选功能）
        if (Full_Flag == 0)
        { // 没有充满
            if (adc_data.UseDat.V_CAP > TEST_LIMIT_CAP_VOLTAGE_VALUE + 1)
            {
                HWFUN_CHG_EN_PIN_DIS;
                Full_Flag = 1; // 标记电量已满
            }
        }
        else
        { // 电量已满
            if (adc_data.UseDat.V_CAP < TEST_LIMIT_CAP_VOLTAGE_VALUE - 1)
            {
                HWFUN_CHG_EN_PIN_EN;
                Full_Flag = 0; // 标记电量已满
            }
            else
            { // 电量已满，不进行功率控制
                continue;
            }
        }
#endif
        // 在收到不充电的指令以后就不再进行充电运算
        if (!Get_Charge_Permission())
        {
            // 使能超级电容充电模块
            HWFUN_CHG_EN_PIN_EN;
            // 1. 计算电源效率
            P_IN_Now = adc_data.UseDat.V_IN * adc_data.UseDat.I_IN;
            Zero_fix(P_IN_Now, 2.0f);
            Power_Est(P_app, P_IN_Now, &Power_State);

            // 2. 计算功率调节理论计算值
            P_err_last = P_err;                       // 记录上次的功率偏差量，用于微分量计算
            P_err = P_IN_Now - P_set;                 // 底盘功率偏差值，超功率为正
            P_err_D = P_err - P_err_last;             //微分量，趋向于超功率为正
            P_fix = -(P_err + P_Fix_D_set * P_err_D); // 超功率时需要对功率进行减小，并按照微分量对调节过程进行阻尼
            if (first_flag)
            {
                P_app = P_set * Power_State.PCE; // 首次运行时不加入Pfix
                first_flag = 0;
            }
            else
            {
                P_app = (P_fix + P_set) * Power_State.PCE; // 计算出功率调节理论计算值
            }

            // 3. 执行功率设定
            P_app = Charge_Set_CHGOUT_P(P_app); // 执行功率设定
        }
        else
        {
            // 设定电流 0
            Charge_Set_CHGOUT_P(0.f);
            // 关闭充电模块
            HWFUN_CHG_EN_PIN_DIS;
        }
    }
}

// 充电控制初始化
int mod_charge_init(float P_Set_ini)
{
    // 先启动模块，开始较低功率充电
    CHG_Set_Curr(MOD_CHG_I_SET_START); // 初始电流1.0A
    // 使能超级电容充电模块
    HWFUN_CHG_EN_PIN_EN;
    // 延时100ms，等待充电模块以及adc采样的工作状态趋于稳定
    rt_thread_delay(100);

    // 使用软件定时器进行周期控制

    /* 初始化用于同步的信号量 */
    // 创建一个动态信号量，初始值是 0
    Charge_Ctrl_Timer_dsem = rt_sem_create("CHG_dsem", 0, RT_IPC_FLAG_FIFO);
    if (Charge_Ctrl_Timer_dsem == RT_NULL)
    {
        rt_kprintf("create Charge_Ctrl_Timer_dsem failed.\n");
        return RT_ERROR;
    }

    /* 初始化软件定时器 */
    Charge_Ctrl_Timer = rt_timer_create("timer1", Charge_Ctrl_Timer_TimeOut,
                                        RT_NULL, 3,
                                        RT_TIMER_FLAG_PERIODIC); // 定时器周期性定时
                                                                 /* 启动定时器 1 */
    if (Charge_Ctrl_Timer != RT_NULL)
        rt_timer_start(Charge_Ctrl_Timer);

    // 初始化相关数据
    adc_DataProcess();
    float V_CAP_REC;
    V_CAP_REC = adc_data.UseDat.V_CAP;
    Zero_fix(V_CAP_REC, 2.0f);
    P_app = MOD_CHG_I_SET_START * V_CAP_REC; // 计算出当前应用的功率
    P_set = P_Set_ini;                       // 初始化功率设定值
    // 初始化控制线程
    Charge_Ctrl_thread_tid = rt_thread_create("P_CTR_TH",
                                              Charge_Ctrl_Thread, RT_NULL,
                                              1024,
                                              CHARGE_P_CTRL_THREAD_PRIO, 2);
    /* 如果获得线程控制块，启动这个线程 */
    if (Charge_Ctrl_thread_tid != RT_NULL)
        rt_thread_startup(Charge_Ctrl_thread_tid);

    return RT_EOK;
}
