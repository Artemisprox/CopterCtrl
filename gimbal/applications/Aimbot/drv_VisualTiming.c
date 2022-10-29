#include "drv_VisualTiming.h"

#include <rthw.h>

// delay到指定tick函数，会通过指针Tick_WakeUp向外输出线程唤醒的实际时刻
rt_err_t rt_thread_delay_to_tick(rt_tick_t Tick_WaitFor, rt_tick_t* Tick_WakeUp)
{
    register rt_base_t level;
    struct rt_thread *thread;
    rt_tick_t DelayTick;

    RT_ASSERT(Tick_WakeUp != RT_NULL);

    /* set to current thread */
    thread = rt_thread_self();
    RT_ASSERT(thread != RT_NULL);
    RT_ASSERT(rt_object_get_type((rt_object_t)thread) == RT_Object_Class_Thread);

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    if (rt_tick_get() < Tick_WaitFor)
    {
        // 需要延时
        DelayTick = Tick_WaitFor - rt_tick_get();

        /* suspend thread */
        rt_thread_suspend(thread);

        /* reset the timeout of thread timer and start it */
        rt_timer_control(&(thread->thread_timer), RT_TIMER_CTRL_SET_TIME, &DelayTick);
        rt_timer_start(&(thread->thread_timer));

        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        rt_schedule();

        /* clear error number of this thread to RT_EOK */
        if (thread->error == -RT_ETIMEOUT)
        {
            thread->error = RT_EOK;
        }
    }
    else
    {
        // 不需要延时
        rt_hw_interrupt_enable(level);
    }

    /* get the wakeup tick */
    *Tick_WakeUp = rt_tick_get();

    return RT_EOK;
}
