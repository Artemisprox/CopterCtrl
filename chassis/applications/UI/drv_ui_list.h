#ifndef DRV_UI_LIST_H
#define DRV_UI_LIST_H

#include "HList.h"
#include "drv_CustomUI_basic.h"

// typedef struct
// {
//     struct list_head list;
//     ui_basic_e (*func)(void *param);
//     int run_times;
//     int period;
// }ui_func_list_t;

// typedef struct
// {
//     struct list_head list;
//     short id;
//     float position;
//     int hurt_time;
//     short hurt_type;
// } ui_enemy_list_t;

typedef struct
{
    ui_basic_e (*func)(void *param);
    int run_times;
    int period;
    int period_cnt;
} ui_func_t;

typedef struct
{
    short id;
    float position;
    int hurt_time;
    short hurt_type;
} ui_enemy_t;

typedef union
{
    ui_func_t func;
    ui_enemy_t enemy;
}ui_member_u;

typedef struct
{
    struct list_head list;
    ui_member_u member;
} ui_list_t;

void UI_List_Init(ui_list_t *head);
void UI_Func_List_Add(ui_list_t *head, ui_basic_e (*func)(void *param), int run_times, int period);
void UI_Enemy_List_Add(ui_list_t *head, short id, float pos, int hurt_time, short hurt_type);
void UI_List_Delete(ui_list_t *entry);
rt_bool_t UI_List_Empty(ui_list_t *head);
void UI_list_Destroy(ui_list_t *head);
#endif
