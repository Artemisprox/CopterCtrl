#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_canthread.h"
#include "mod_RefSystem.h"
#include "SuperCap_Com.h"
#include "drv_GimbalCom.h"
#include "CustomUI.h"
#include "app_ChassisCtrl.h"
#include "mod_Monitor.h"
#include "func_MonHandling.h"

#if (!defined CORE_USING_INFANTRY) && (!defined CORE_USING_HERO)
#error "Please specify the robot type!"
#endif

int main(void)
{
    // 上电提示音
    set_buzzer(2000);
    rt_thread_mdelay(120);
    set_buzzer(0);

    if (MONITOR_INIT() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);

    /* SoftMod */
    if (Can1_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
    if (Can2_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
    if (rt_thread_mdelay(150) != RT_EOK) // CAN 启动延时
        Robot_Chassis_Reset(RT_TRUE);

    /* HardMod */

    /* Communication */
    if (DJI_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
    if (Scpr_Com_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
    if (Gimbal_Com_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);

    /* Main Module */
    if (Chassis_Ctrl_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
    if (UI_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);

    return RT_EOK;
}
