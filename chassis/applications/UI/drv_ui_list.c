#include "drv_ui_list.h"
#include <rtthread.h>

/***
* @brief    ui链表初始化
* @param    head:链表头指针所在结构体
* @retval   none
* @author   dxy
***/
void UI_List_Init(ui_list_t *head)
{
    INIT_LIST_HEAD(&head->list);
}

/***
* @brief
* @param    entry:删除的节点
* @retval   none
* @author   dxy
***/
void UI_List_Delete(ui_list_t *entry)
{
    list_del(&entry->list);
    rt_free(entry);
}

/***
* @brief
* @param    entry:删除的节点
* @retval   none
* @author   dxy
***/
rt_bool_t UI_List_Empty(ui_list_t *head)
{
    if(list_empty(&head->list))
        return RT_TRUE;
    else
        return RT_FALSE;
}

/***
* @brief    销毁ui整条链表
* @param    head:链表头指针所在结构体
* @retval   none
* @author   dxy
***/
void UI_list_Destroy(ui_list_t *head)
{
    ui_list_t *pos, *n;
    list_for_each_entry_safe(pos,n,&head->list,list)
    {
        list_del(&pos->list);
        rt_free(pos);
    }
}

/***
* @brief    添加ui绘制函数
* @param    head:链表头指针所在结构体
* @param    func:UI绘制函数
* @param    run_times:  该UI绘制次数    n=0:一直绘制   n>0:绘制n次
* @param    period:     运行周期,每循环n次绘制一次, 注:只绘制一次的图形要设成1,不然重置UI的时候有概率无法显示
* @retval   none
* @author   dxy
***/
void UI_Func_List_Add(ui_list_t *head, ui_basic_e (*func)(void *param), int run_times, int period)
{
    ui_list_t *p = (ui_list_t *)rt_malloc(sizeof(ui_list_t));
    p->member.func.func = func;
    p->member.func.run_times = run_times;
    p->member.func.period = period;
    list_add_tail(&p->list, &head->list);
}

/***
* @brief    添加敌方信息链表,用于绘制敌人方位
* @param    head:链表头指针所在结构体
* @param    id:自定义编号,0-8
* @param    pos: 敌方相对于云台的方位, 单位°
* @param    hurt_time : 记录扣血的时间
* @param    hurt_type : 伤害类型,由裁判系统读出
* @retval   none
* @author   dxy
***/
void UI_Enemy_List_Add(ui_list_t *head, short id, float pos, int hurt_time, short hurt_type)
{
    ui_list_t *p = (ui_list_t *)rt_malloc(sizeof(ui_list_t));
    p->member.enemy.id = id;
    p->member.enemy.position = pos;
    p->member.enemy.hurt_time = hurt_time;
    p->member.enemy.hurt_type = hurt_type;
    list_add_tail(&p->list, &head->list);
}

