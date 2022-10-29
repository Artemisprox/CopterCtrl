#include "app_LED_Show.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "mod_adc.h"
#include "drv_thread.h"

#include "func_HW_Pin_Set.h"

// 此文件控制板载LED显示充电功率和电容电量

// 充电功率通过五轴按键旁的LED的亮度进行显示

// 电容电量通过4个LED亮起的个数进行显示

static rt_thread_t LED_Show_App_Thread_tid = RT_NULL; // OLED显示控制线程句柄
static Cap_Energy_Type CAP_Energy_data;

static char LED_4_REM;

// 对零散的LED-GPIO操作进行数字化的封装
static void LED_4_GPIOFUN(char LED_Num, rt_base_t LED_State)
{
    rt_base_t LED_GPIO;
    switch (LED_Num)
    {
    case 1:
        LED_GPIO = HW_LED0_1_PIN;
        break;
    case 2:
        LED_GPIO = HW_LED0_2_PIN;
        break;
    case 3:
        LED_GPIO = HW_LED0_3_PIN;
        break;
    case 4:
        LED_GPIO = HW_LED0_4_PIN;
        break;

    default:
        return;
    }
    rt_pin_write(LED_GPIO, LED_State);
}

//控制4个LED灯亮起的个数
static void LED_4_CTRL(int LED_Num)
{
    int LED_Select;
    int Bit;
    int fori;

    // 快捷计算需要亮起的灯 用二进制表示
    LED_Select = (1 << LED_Num);
    LED_Select--;

    Bit = 1; // 第一位置1

    for (fori = 0; fori < 4;fori++)
    {
        if ((LED_Select & Bit)!=0)
        { //如果需要亮起第一个灯
            if ((LED_4_REM & Bit) == 0)
            { //需要电平翻转
                LED_4_GPIOFUN(fori + 1, HW_LED_ON);
                LED_4_REM |= Bit;
            }
        }
        else
        {
            if ((LED_4_REM & Bit) != 0)
            { //需要电平翻转
                LED_4_GPIOFUN(fori + 1, HW_LED_OFF);
                LED_4_REM &= ~Bit;
            }
        }
        Bit <<= 1;//开始处理下一个灯
    }
}

static rt_int16_t Energy_Count;//用于放置计算好的LED亮起个数
// 完成电容电量的获取和LED灯的控制
static void LED_CAP_Energy_Fresh(void)
{
    // 获取电容电量数据
    Get_CAP_Energy(&CAP_Energy_data);
    Energy_Count = ((rt_int16_t)(CAP_Energy_data.Energy_Percentage_Real * 4.0f+0.5f));

    // 限幅
    if(Energy_Count>4)
    {
        Energy_Count = 4;
    }
    else if(Energy_Count<0)
    {
        Energy_Count = 0;
    }

    // 执行显示
    LED_4_CTRL(Energy_Count);
}

static rt_int16_t LED_ON_Time, LED_OFF_Time;
//LED灯效控制线程函数-显示充电功率
static void LED_Show_Power_Thread(void *Para)
{
    rt_thread_delay(200);//上电后先延时，保证其它程序模块正常启动后再控制灯效
    while(1)
    {
        LED_ON_Time = (rt_int16_t)(adc_data.UseDat.I_IN / 4.0f * LED_SHOW_SOFTPWM_PERIOD_MS);
        if (LED_ON_Time > LED_SHOW_SOFTPWM_PERIOD_MS)
        {
            LED_ON_Time = LED_SHOW_SOFTPWM_PERIOD_MS;
        }
        else if (LED_ON_Time < 0)
        {
            LED_ON_Time = 0;
        }
        
        LED_OFF_Time = LED_SHOW_SOFTPWM_PERIOD_MS - LED_ON_Time;

        if (LED_ON_Time!=0)
        {
            HWFUN_LEDKEY_ON;
            rt_thread_delay(LED_ON_Time);
        }

        if (LED_OFF_Time!=0)
        {
            HWFUN_LEDKEY_OFF;
            rt_thread_delay(LED_OFF_Time);
        }
    }
}

//LED灯效控制线程函数-显示电量
static void LED_Show_Energy_Thread(void *Para)
{
    rt_thread_delay(200); //上电后先延时，保证其它程序模块正常启动后再控制灯效
    while(1)
    {
        LED_CAP_Energy_Fresh();
        rt_thread_delay(20);
    }
}

//初始化LED显示控制线程
static int LED_Show_App_Init(void)
{
    //清空LED状态记录数据
    LED_4_REM = 0;

    //启动显示控制线程
    LED_Show_App_Thread_tid = rt_thread_create("LED_ENGY",
                                                LED_Show_Energy_Thread, RT_NULL,
                                                512,
                                                LED_SHO_APP_THREAD_PRIO, 1);
    /* 启动这个线程 */
    rt_thread_startup(LED_Show_App_Thread_tid);

    //启动显示控制线程
    LED_Show_App_Thread_tid = rt_thread_create("LED_POWR",
                                               LED_Show_Power_Thread, RT_NULL,
                                               512,
                                               LED_SHO_APP_THREAD_PRIO, 1);
    /* 启动这个线程 */
    rt_thread_startup(LED_Show_App_Thread_tid);
    return RT_EOK;
}
INIT_APP_EXPORT(LED_Show_App_Init);
