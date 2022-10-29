#ifndef __DRV_VISUALTIMING_H__
#define __DRV_VISUALTIMING_H__

#include "rtthread.h"

// delay到指定tick函数，会通过指针Tick_WakeUp向外输出线程唤醒的实际时刻
extern rt_err_t rt_thread_delay_to_tick(rt_tick_t Tick_WaitFor, rt_tick_t *Tick_WakeUp);

#endif
