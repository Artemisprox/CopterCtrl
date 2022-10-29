#ifndef __DRV_MONITOR_H__
#define __DRV_MONITOR_H__

#include <rtdef.h>

#define SWDG_INITED_FLAG (0x5A)

/*看门狗对象ID，要求id唯一，尽量小*/
typedef enum
{
    SWDG_IMU_ID = 0,
    SWDG_TEMPCTRL_ID,
    SWDG_GIMBAL_ID,
    SWDG_CAN1_ID,
    SWDG_CAN2_ID,
    SWDG_AIMBOT_SEND_ID,
    SWDG_GUNDATA_ID,
    SWDG_STRIKE_ID,
    SWDG_ROBOCONTROL_ID,
    MONITOR_ID_ALL,
} swdg_deviceID;

/*要拓展更多的RGB报警颜色需要在RGB驱动文件.h增加三元色宏；
在Alarm_Thread线程的swicth语句内增加case选项；*/
typedef enum
{
    ALARM_WHITE,
    ALARM_RED,
    ALARM_BLUE,
    ALARM_GREEN,
    ALARM_YELLOW,
    ALARM_PURPLE,
    ALARM_BROWN,
    ALARPINK

} Alarm_color_e;

/*这里采用链表套结构体的方法，优点：思路简单易实现；缺点：通用性不够*/
//看门狗句柄
typedef struct swdg_dev
{
    swdg_deviceID ID;           // 小看门狗跟踪设备的id，id是唯一的
    Alarm_color_e color;        // 监视器报警的RGB灯颜色
    rt_uint32_t time_threshold; // 时间阈值，超过该时间未响应则报警，单位ms
    rt_tick_t time_deadline;    // 报警时刻, 到这个时刻不喂狗就直接报警

    rt_uint8_t flag_inited; // 写入 0x20 代表初始化成功, 反之表示未初始化
    rt_bool_t if_start;     // 看门狗开始工作
    rt_bool_t if_error;     // RT_FALSE:正常，RT_TRUE:异常
    rt_bool_t if_alarm;     // RT_TRUE:启用报警，RT_FALSE:关闭报警功能

    struct swdg_dev *next;         // 指向下一个要监视的看门狗句柄
    rt_err_t (*handle)(rt_bool_t); // 异常对应的处理函数指针,(触发式函数，不会一直轮询)

} swdg_dev_t;

//报警双向链表，挂载异常看门狗对象
typedef struct mlist_node
{
    struct mlist_node *next;
    struct mlist_node *prev;
    swdg_dev_t *swdg;

} alarm_dev_t;

/**
 * @brief  创建一个软件看门狗对象
 * @param  id    监视ID
 * @param  color  报警颜色
 * @param  if_alarm  是否启用报警功能
 * @param  time_threshold    报警时间，超过该时间不喂狗则会报警（单位ms）
 * @param  handle    异常处理函数指针
 * @param  init_flag 初始化用的标志位
 * @return RT_ERROR  初始化失败，RT_EOK  初始化成功
 * @author mqy
 */
rt_err_t Swdg_Create(swdg_deviceID id, Alarm_color_e color, rt_bool_t if_alarm,
                     rt_uint32_t time_threshold, rt_err_t (*handle)(rt_bool_t), rt_uint8_t init_flag);

/**
 * @brief initialize a list
 * @param l list to be initialized
 */
void Mlist_Init(alarm_dev_t *l);

/***
 * @brief  insert a node after alarm_head
 * @param  HungryDog   没有及时被喂狗的swdg指针
 * @return none
 * @author Lvfp
 ***/
void Mlist_Insert(swdg_dev_t *HungryDog);

/***
 * @brief  remove node from list and release memory block
 * @param  mID: 对应监视器的ID
 * @return None
 * @author Lvfp
 ***/
void Mlist_Remove(swdg_deviceID mID);

#endif
