#ifndef __MOD_MONITOR_H__
#define __MOD_MONITOR_H__
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_Monitor.h"
#include "func_Alarm.h"
#ifdef BSP_USING_WDT
    #include "drv_HardWdt.h"
#endif

/*线程周期*/
#define MONITOR_PERIOD	2	//设定监控周期，单位ms


/*带宏开关的对�?�接�?*/ 
#if  defined(CORE_USING_MONITOR)
	/*监�?�器初�?�化，建�?用这�?而不�?初�?�化函数*/
	#define MONITOR_INIT()	Monitor_Init()
    /* �?动一�?指定监�?�器 */
    #define SWDG_START(ID) Swdg_Start(ID)
	/*�?通喂�?*/
    #define SWDG_FEED(ID)   Swdg_Feed(ID)
	/*ID，看门狗id；condition == 1时，喂狗*/
	#define SWDG_IF_FEED(ID, condition)   do{ 					\
											if(condition)		\
												Swdg_Feed(ID);	\
											} while(0);
#else
	#define MONITOR_INIT()  RT_EOK
    #define SWDG_FEED(ID)    
    #define SWDG_START(ID)
	#define SWDG_IF_FEED(ID, condition) 
#endif



/**
* @brief    该函数初始化监�?�器
* @param    None
* @return   RT_ERROR：初始化失败
* @author   mqy
*/
rt_err_t Monitor_Init(void);

/**
* @brief    给看门狗喂�??(移出ID对应的报警节�?,复位剩余时间)
* @param    mID 看门狗id
* @return   None
* @author   mqy
*/
void Swdg_Feed(swdg_deviceID mID);

/**
* @brief    查�?�看门狗对象�?否异�?
* @param    mID 看门狗id
* @return   RT_FALSE：�?�常，RT_TRUE：异�?
* @author   lfp
*/
rt_bool_t Swdg_If_Error(swdg_deviceID mID);

/**
 * @brief �?动一�?监�?�器
 * @author fwlh
 * @param  mID              待启动监视器�? ID
 */
extern void Swdg_Start(swdg_deviceID mID);
	
	
#endif
