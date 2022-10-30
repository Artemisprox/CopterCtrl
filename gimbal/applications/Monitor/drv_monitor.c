#include "drv_monitor.h"
#include <rtthread.h>

//小看门狗链表头指针,Head pointer
swdg_dev_t *monitor_hp = RT_NULL;
static rt_sem_t monitor_sem = RT_NULL; // 用来对操作链表上锁的信号量
//报警线程双向链表头
alarm_dev_t alarm_head;
//取头指针
alarm_dev_t *alarm_hp = &alarm_head;
static rt_sem_t alarm_sem = RT_NULL; // 用来对操作链表上锁的信号量

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
                     rt_uint32_t time_threshold, rt_err_t (*handle)(rt_bool_t), rt_uint8_t init_flag)
{
    // 判断信号量是不是存在
    if (!monitor_sem)
    {
        monitor_sem = rt_sem_create("MonitorSem", 1, RT_IPC_FLAG_FIFO);
        if (!monitor_sem)
            return RT_ERROR;
    }

    swdg_dev_t *swdg_dev_tem = (swdg_dev_t *)rt_malloc(sizeof(swdg_dev_t)); //申请空间
    if (swdg_dev_tem == RT_NULL)
        return RT_ERROR;

    // 操作前第一步是上锁
    rt_sem_take(monitor_sem, RT_WAITING_FOREVER);
    while (RT_EOK == rt_sem_trytake(monitor_sem))
        continue;

    //构建链表
    swdg_dev_tem->next = monitor_hp;
    monitor_hp = swdg_dev_tem;

    //赋值
    swdg_dev_tem->flag_inited = init_flag;
    swdg_dev_tem->ID = id;
    swdg_dev_tem->color = color;
    swdg_dev_tem->if_error = RT_FALSE;
    swdg_dev_tem->if_alarm = if_alarm;
    swdg_dev_tem->time_threshold = time_threshold;
    swdg_dev_tem->time_deadline = rt_tick_get() + time_threshold;
    swdg_dev_tem->handle = handle;
    swdg_dev_tem->if_start = RT_FALSE;

    // 操作完成, 解锁
    rt_sem_release(monitor_sem);
    return RT_EOK;
}

/**
 * @brief initialize a list
 * @param l list to be initialized
 */
void Mlist_Init(alarm_dev_t *l)
{
    l->next = l;
    l->prev = l;
    l->swdg = RT_NULL;
}

/***
 * @brief  insert a node after alarm_head
 * @param  HungryDog   没有及时被喂狗的swdg指针
 * @return none
 * @author Lvfp
 ***/
void Mlist_Insert(swdg_dev_t *HungryDog)
{
    // 判断信号量是不是存在
    if (!alarm_sem)
    {
        alarm_sem = rt_sem_create("AlarmSem", 1, RT_IPC_FLAG_FIFO);
        if (!alarm_sem)
            RT_ASSERT(0);
    }

    alarm_dev_t *alarm_dev_tem = alarm_hp->next;

    //确保挂在alarm_hp的链表成员的看门狗ID不重复
    while (alarm_dev_tem != alarm_hp)
    {
        if (HungryDog->ID == alarm_dev_tem->swdg->ID)
            return;
        alarm_dev_tem = alarm_dev_tem->next;
    }

    // 操作前第一步是上锁
    rt_sem_take(alarm_sem, RT_WAITING_FOREVER);
    while (RT_EOK == rt_sem_trytake(alarm_sem))
        continue;

    //申请空间，挂上指针
    alarm_dev_t *n = (alarm_dev_t *)rt_malloc(sizeof(alarm_dev_t));
    n->swdg = HungryDog;

    //建立alarm_hp后一个节点与新节点的联系
    alarm_hp->next->prev = n;
    n->next = alarm_hp->next;
    //建立alarm_hp与新节点的联系
    alarm_hp->next = n;
    n->prev = alarm_hp;

    // 操作完以后解锁
    rt_sem_release(alarm_sem);
}

/***
 * @brief  remove node from list and release memory block
 * @param  mID: 对应监视器的ID
 * @return None
 * @author Lvfp
 ***/
void Mlist_Remove(swdg_deviceID mID)
{
    // 判断信号量是不是存在
    if (!alarm_sem)
    {
        alarm_sem = rt_sem_create("AlarmSem", 1, RT_IPC_FLAG_FIFO);
        if (!alarm_sem)
            RT_ASSERT(0);
    }

    alarm_dev_t *n = RT_NULL;
    alarm_dev_t *alarm_dev_tem = alarm_hp->next;

    //索引找到ID对应的报警链表的节点
    while (alarm_dev_tem != alarm_hp)
    {
        if (mID == alarm_dev_tem->swdg->ID)
        {
            n = alarm_dev_tem;
            break;
        }
        alarm_dev_tem = alarm_dev_tem->next;
    }

    if (n == RT_NULL)
        return; //无效id

    // 操作前第一步是上锁
    rt_sem_take(alarm_sem, RT_WAITING_FOREVER);
    while (RT_EOK == rt_sem_trytake(alarm_sem))
        continue;

    n->next->prev = n->prev;
    n->prev->next = n->next;

    // 操作完以后解锁
    rt_sem_release(alarm_sem);

    // n->next = n->prev = n;
    rt_free(n);
}
