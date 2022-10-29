#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "drv_monitor.h"

#define WDT_DEVICE_NAME "wdt" /* 看门狗设备名称 */

static rt_device_t wdg_dev = NULL; /* 看门狗设备句柄 */

// 喂狗函数
void WDT_Feed(void)
{
#ifdef _WDG_ENABLE_
    /* 在空闲线程的回调函数里喂狗 */
    if (wdg_dev!=NULL) // 防止出现初始化前错误操作的情况
        rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
#else
    return;
#endif
}

// 初始化硬件看门狗
int WDT_Init()
{
#ifdef _WDG_ENABLE_
    rt_err_t ret = RT_EOK;
    rt_uint32_t timeout = 1; /* 溢出时间，单位：秒 */

    /* 根据设备名称查找看门狗设备，获取设备句柄 */
    wdg_dev = rt_device_find(WDT_DEVICE_NAME);
    if (!wdg_dev)
    {
        rt_kprintf("find %s failed!\n", WDT_DEVICE_NAME);
        return RT_ERROR;
    }

    /* 设置看门狗溢出时间 */
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
    if (ret != RT_EOK)
    {
        rt_kprintf("set %s timeout failed!\n", WDT_DEVICE_NAME);
        return RT_ERROR;
    }
    /* 启动看门狗 */
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    if (ret != RT_EOK)
    {
        rt_kprintf("start %s failed!\n", WDT_DEVICE_NAME);
        return -RT_ERROR;
    }

    return ret;
#else
    return 0;
#endif
}
