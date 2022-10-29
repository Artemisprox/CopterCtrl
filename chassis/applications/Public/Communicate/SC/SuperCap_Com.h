#ifndef __SUPERCAP_COM_H__
#define __SUPERCAP_COM_H__
#include <rtdevice.h>

#define SC_PERIOD 10 //超级电容线程轮询周期，单位ms

/**
* @brief    初始化与超级电容端通信
* @param [in]	无
* @return   true:初始化成功	false:初始化失败
* @author   LvFp
*/
rt_err_t Scpr_Com_Init(void);

/***
* @brief    处理超级电容端发送报文
* @param    msg can2报文
* @return   None
***/
void Refresh_Scprdata(struct rt_can_msg* msg);

/**
 * @brief 获取超级电容控制板当前是否在线
 * @author fwlh
 * @return rt_bool_t        超级电容控制板在线返回真
 */
extern rt_bool_t Read_Super_Capacity_Online(void);

/***
* @brief    获得超级电容剩余电量
* @param    None
* @return   剩余电容量,0~100。100为满电
***/
float Get_RemainCapcity(void);


#endif
