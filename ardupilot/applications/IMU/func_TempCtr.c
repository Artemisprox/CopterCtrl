/*
        IMU温度控制
*/
#include "func_TempCtr.h"
#include <rtdevice.h>
#include "drv_thread.h"
#include <board.h>
#include "func_SensorRAW.h"
#include "mod_Monitor.h"
#include <arm_math.h>

static rt_thread_t temp_ctr = RT_NULL;

struct rt_semaphore temp_pid_sem;

TempCTR_t HERO_TPctr;
int16_t IMUTempSet = 45; // 温度设定值

// IMU控温PWM
static struct rt_device_pwm *IMUtemp_pwm = RT_NULL;
#define IMUTEMP_PWMCHANNEL 1

int k_id = 70;           // pid中 用d补偿i 的系数
float ivalue_init = 350; // ivalue叠加的常值

/**
 * @brief 控温专用的PID
 * @param [pid_t*] target：pid结构体
 * @param [float] Error：偏差量
 * @return 无
 * @author zzj
 */
static void PIDTemp_Calculate(pid_t *target, float Error)
{
    float pid_d;
    target->err_old = target->err;
    target->err = Error;

    pid_d = (target->err - target->err_old); // 计算d

    if (target->I_Dis == 0)
    {
        target->i_value += target->ki * Error;
        target->i_value += k_id * pid_d;
        //积分限幅
        if (target->i_value < 0)
        {
            if (target->i_value < -target->i_limit)
                target->i_value = -target->i_limit;
        }
        else
        {
            if (target->i_value > target->i_limit)
                target->i_value = target->i_limit;
        }
    }

    target->out = target->kp * Error + (target->i_value + ivalue_init) + target->kd * pid_d;

    //输出限幅
    if (target->out > target->out_limit_up)
        target->out = target->out_limit_up;
    else if (target->out < target->out_limit_down)
        target->out = target->out_limit_down;
}

int IfTempOK = RT_ERROR;
/**
 * @brief：IMU温度控制线程
 * @param [in]	parameter:该参数不会被使用
 * @return：		无
 * @author：zzj
 */
static void TempCTR_thread(void *parameter)
{
    float Error;
    int ifsemOK;
    static int TempCTR_count = 0;
//    SWDG_START(SWDG_TEMPCTRL_ID);
    while (1)
    {
        /* 读取信号量 */
//        SWDG_FEED(SWDG_TEMPCTRL_ID);
        ifsemOK = rt_sem_take(&temp_pid_sem, 1400);

        HERO_TPctr.TempCTR_pid.set = IMUTempSet; // 更新设定值

        Error = HERO_TPctr.TempCTR_pid.set - Sensor_RAW.Temperature;
        PIDTemp_Calculate(&HERO_TPctr.TempCTR_pid, Error);

        rt_pwm_set(IMUtemp_pwm, IMUTEMP_PWMCHANNEL, 1 * 1000 * 1000, (rt_uint32_t)HERO_TPctr.TempCTR_pid.out * 1000);

        if (TempCTR_count < 2)
        { // 温度还没稳定
            if (fabsf(Error) < 0.5f)
                TempCTR_count++;
            else
            {
                TempCTR_count--;
                if (TempCTR_count < 0)
                    TempCTR_count = 0;
            }
        }
        else
            // 温度已稳定达到设定值
            IfTempOK = RT_EOK;

        if (ifsemOK == -RT_ETIMEOUT)
        { // 由于超时进入的该线程
//            SWDG_FEED(SWDG_TEMPCTRL_ID);
            rt_thread_delay(1000);
        }
    }
}

/**
* @brief：初始化IMU的控温PWM（TIME10 CH1 PF6）
* @param [in] 无
* @return：RT_EOK：初始化成功
                RT_ERROR：初始化失败
* @author：zzj
*/
int IMUtempPWM_init(void)
{
    //尝试查找设备，查找失败时返回
    IMUtemp_pwm = (struct rt_device_pwm *)rt_device_find("pwm10");
    if (IMUtemp_pwm == RT_NULL)
        return RT_ERROR;
    rt_pwm_set(IMUtemp_pwm, IMUTEMP_PWMCHANNEL, 1000000, 0);
    rt_pwm_enable(IMUtemp_pwm, IMUTEMP_PWMCHANNEL);

    return RT_EOK;
}
INIT_APP_EXPORT(IMUtempPWM_init);

/**
* @brief：初始化IMU温度控制线程
* @param [in]	无
* @return：		1:初始化成功
                0:初始化失败
* @author：zzj
*/
int TempCTR_init(void)
{
    pid_init(&HERO_TPctr.TempCTR_pid, 300, 15, 500, 250, 1000, 0);

    //初始化IMU温度控制线程
    temp_ctr = rt_thread_create(
        "TP_CTR",            //线程名
        TempCTR_thread,      //线程入口
        RT_NULL,             //入口参数无
        4096,                //线程栈
        THREAD_PRIO_TEMPCTR, //线程优先级
        1);                  //线程时间片大小

    //线程创建失败返回false
    if (temp_ctr == RT_NULL)
        return RT_ERROR;

    //线程启动失败返回false
    if (rt_thread_startup(temp_ctr) != RT_EOK)
        return RT_ERROR;

    return RT_EOK;
}
