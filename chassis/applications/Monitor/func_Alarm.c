#include "func_Alarm.h"
#include "drv_Monitor.h"
#include "HThread_data.h"

extern alarm_dev_t *alarm_hp;

/**
 * @brief    报警线程
 * @param    无
 * @return   无
 * @author   Lvfp
 */
static void Alarm_Thread(void *parameter)
{
    while (1)
    {
        alarm_dev_t *alarm_dev_tem = alarm_hp->next;

        while (alarm_dev_tem != alarm_hp) //索引
        {
            //是否报警
            if (alarm_dev_tem->swdg->if_alarm == RT_TRUE)
            {
#ifdef BSP_USING_RGB_LIGHT
                // RGB灯报警，多个异常看门狗，则轮流显示对应的颜色
                switch (alarm_dev_tem->swdg->color)
                {
                case ALARM_WHITE:
                    set_RGB(CORE_WHITE);
                    break;
                case ALARM_RED:
                    set_RGB(CORE_RED);
                    break;
                case ALARM_BLUE:
                    set_RGB(CORE_BLUE);
                    break;
                case ALARM_GREEN:
                    set_RGB(CORE_GREEN);
                    break;
                case ALARM_YELLOW:
                    set_RGB(CORE_YELLOW);
                    break;
                case ALARM_PURPLE:
                    set_RGB(CORE_PURPLE);
                    break;
                case ALARM_BROWN:
                    set_RGB(CORE_BROWN);
                    break;
                }
#endif
#ifdef BSP_USING_BUZZER
                //通过拨码开关关闭or开启蜂鸣器
                if (rt_pin_read(SWITCH_BUZZER_PIN))
                {
                    // ID值多少就响几次
                    for (int i = 0; i < alarm_dev_tem->swdg->ID; i++)
                    {
                        ALARM_SET
                    }
                    ALARM_RESET //不同音调，区分多个ID
                }
#endif

            } // if
            alarm_dev_tem = alarm_dev_tem->next;
        }

        //如果没有异常看门狗，关闭RGB和蜂鸣器
        if (alarm_hp == alarm_hp->next)
        {
#ifdef BSP_USING_RGB_LIGHT
            set_RGB(CORE_BLACK);
#endif
#ifdef BSP_USING_BUZZER
            set_buzzer(0);
#endif
        }

        rt_thread_mdelay(ALARM_PERIOD);
    } // while(1)
}

/**
 * @brief    报警初始化
 * @param    无
 * @return   初始化成功or失败
 * @author   Lvfp
 */
rt_err_t Alarm_Init(void)
{
    rt_thread_t alarm_device = RT_NULL; //报警线程句柄

    //拨码开关控制
    rt_pin_mode(67, PIN_MODE_INPUT_PULLDOWN);

    //初始化报警双向链表和创建所有监视器对象
    Mlist_Init(alarm_hp);

    //报警线程创建
    alarm_device = rt_thread_create(
        "alarm",
        Alarm_Thread,
        RT_NULL,
        THREAD_STACK_ALARM,
        THREAD_PRIO_ALARM,
        THREAD_TICK_ALARM);

    //查看是否创建成功
    if (alarm_device != RT_NULL)
        if (rt_thread_startup(alarm_device) != RT_EOK)
            return RT_ERROR;
    else
        return RT_ERROR;

    return RT_EOK;
}
