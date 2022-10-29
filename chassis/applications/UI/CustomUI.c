#include "CustomUI.h"
#include "func_CustomUI.h"
#include "app_GetGim.h"
#include "app_GetRef.h"
#include "HThread_data.h"
#include "drv_ui_list.h"
#include "mod_Monitor.h"
#include "drv_GimMotor.h"
#include "drv_wheel.h"
#include "SuperCap_Com.h"

static struct rt_timer task_50ms;
static struct rt_semaphore custom_ui_50ms_sem; //定时触发
struct rt_semaphore custom_ui_sem;             //用于ui函数内部,保护上下文
struct rt_mutex custom_ui_mutex;               //用于ui函数内部,保护上下文
static ui_list_t UI_list = {LIST_HEAD_INIT(UI_list.list)};
static ui_list_t *UI_list_rec;
static rt_uint8_t ui_loop_flag; //完成一次循环或成功发送一次后置1

static struct UI_Data_t
{
    strike_mode_e Strike_Mode;                          // 发射机构模式
    ui_motion_mode_e Motion_Mode;                       // 底盘模式
    ui_aimbot_mode_e Aimbot_Mode;                       // 瞄准模式
    My_Color_Enum Color_Myself;                         // 当前自瞄的己方颜色
    ui_option_display_e MenuOption;                     // 当前所在的菜单
    Visual_Error_State_Enum Visual_Error_State;         // 视觉异常状态
    Local_AmmoBooster_Mode_Enum Local_AmmoBooster_Mode; // 当前设置的本地发射机构类型(只有裁判系统离线的时候画出来)
    Local_Chassis_Mode_Enum Local_Chassis_Mode;         // 当前设置的本地底盘类型(只有裁判系统离线的时候画出来)
    rt_uint8_t Local_Robot_Level;                       // 当前设置的本地机器人等级(只有裁判系统离线的时候画出来)
    rt_uint8_t Gimbal_Motor_Offline;                    // 云台电机离线情况,从低到高分别为：Yaw、Pitch、右摩擦轮、左摩擦轮、播弹盘, 置 1 表示离线
    rt_uint8_t Chassis_Motor_Offline;                   // 底盘电机离线情况, 从低到高分别为：右前、左前、左后、右后、Yaw轴电机, 置 1 表示离线
    float RemainCapacity;                               // 超级电容当前的剩余电量

    unsigned Client_Mode : 1;              // 当前处于客户端控制模式
    unsigned Rub_Started : 1;              // 摩擦轮已开启
    unsigned HeatLimit_Status : 1;         // 本地热量限制状态, 1 代表关闭
    unsigned PowerRestriction_Status : 1;  // 自动电量保留状态, 1 代表开启自动电量保留
    unsigned Capacity_Charge_Cmd : 1;      // 开启超级电容充电的指令, 1 为开启充电(0时建议标红)
    unsigned RefSystem_Forced_Offline : 1; // 裁判系统强制离线指令, 1 代表强制离线
    unsigned Gimbal_Online : 1;            // 云台通信正常, 1 代表通信正常
    unsigned Super_Capacity_Online : 1;    // 超级电容控制板通信正常, 1 代表通信正常
    unsigned Chassis_Reset_Warning : 1;    // 底盘复位警示, 1 代表底盘刚刚发生了复位
    unsigned Gimbal_Reset_Warning : 1;     // 云台复位警示, 1 代表云台刚刚发生了复位, 需要告诉操作手及时设置机器人状态
    unsigned RefSystem_Offline : 1;        // 裁判系统数据离线标志, 1 代表裁判系统数据离线
    unsigned Strike_Stuck : 1;             // 发射机构卡弹标志位, 1 代表发射机构卡弹
    unsigned Robot_Hurted : 1;             // 机器人收到伤害
    unsigned UI_Reset_Flag : 1;            // 重置 UI 的标志位, 翻转时需要重置
} UI_Data;                                 // 用于进行 UI 绘制的相关数据

/**
 * @brief 获取绘制 UI 需要用到的数据
 * @author fwlh
 */
static void Get_UI_Data(void)
{
    UI_Data.Strike_Mode = Get_StrikeMode();
    UI_Data.Motion_Mode = Get_Motion_Mode();
    UI_Data.Aimbot_Mode = Get_Aimbot_Mode();
    UI_Data.Color_Myself = Get_Color_Myself();
    UI_Data.MenuOption = Get_UI_Option_Flag();
    UI_Data.Visual_Error_State = Get_Visual_Working_Error_State();
    UI_Data.Local_AmmoBooster_Mode = Get_Local_AmmoBooster_Type();
    UI_Data.Local_Chassis_Mode = Get_Local_Chassis_Type();
    UI_Data.Local_Robot_Level = Get_Local_Robot_Level();
    UI_Data.Gimbal_Motor_Offline = Get_Gimbal_Motor_Offline_State();
    UI_Data.Chassis_Motor_Offline = (Read_Wheels_Offline_State() | ((Get_Yaw_Motor_Offline_State() & 0x01) << 4));
    UI_Data.Client_Mode = Get_Client_Status();
    UI_Data.Rub_Started = Read_Rub_Started();
    UI_Data.HeatLimit_Status = Get_HeatLimit_Status();
    UI_Data.PowerRestriction_Status = Get_PowerRestriction_Status();
    UI_Data.Capacity_Charge_Cmd = Get_Charge_Cmd();
    UI_Data.RefSystem_Forced_Offline = Get_Ref_Offline_Cmd();
    UI_Data.Gimbal_Online = (Get_Gim_FreshTick()) && (rt_tick_get() - Get_Gim_FreshTick() < 200);
    UI_Data.Super_Capacity_Online = Read_Super_Capacity_Online();
    UI_Data.Chassis_Reset_Warning = (rt_tick_get() < 3000);
    UI_Data.Gimbal_Reset_Warning = Get_Gim_Reset_Status();
    UI_Data.RefSystem_Offline = (!RefReceiveTime.power_heat_data) || (!RefReceiveTime.game_robot_state) ||
                                (rt_tick_get() - RefReceiveTime.power_heat_data > 1000) || (rt_tick_get() - RefReceiveTime.game_robot_state > 1000);
    UI_Data.Strike_Stuck = Get_Gimbal_Stuck_State();
    UI_Data.Robot_Hurted = Ref_Get_Robot_If_Hurt();
    UI_Data.UI_Reset_Flag = Get_UI_Reset_Flg();
}

void UI_Func_List_Init(void);

void CustomUI_Update_Thread(void *parameter)
{
    rt_uint8_t ui_reset = Get_UI_Reset_Flg();
    static rt_uint8_t ui_reset_last;
    static ui_list_t *UI_list_pos, *UI_list_n;
    static rt_uint32_t period_cnt = 0;
    ui_reset_last = ui_reset;
    UI_list_rec = &UI_list;
    while (1)
    {
        SWDG_FEED(SWDG_UI_ID);
        ui_reset = Get_UI_Reset_Flg();
        if (ui_reset_last != ui_reset)
        {
            ui_reset_last = ui_reset;
            UI_Func_List_Init(); // UI重置
            period_cnt = 0;
        }
        list_for_each_entry_safe(UI_list_pos, UI_list_n, &UI_list.list, list)
        {
            if (UI_list_rec == UI_list_pos) //回到循环链表起点,结束循环发送并数据
            {
                rt_mutex_release(&custom_ui_mutex);
                rt_sem_take(&custom_ui_sem, RT_WAITING_FOREVER);
                rt_mutex_take(&custom_ui_mutex, RT_WAITING_FOREVER);
            }
            if (ui_loop_flag == 1) //完成一次循环或已经发送一次数据
            {
                UI_list_rec = UI_list_pos; //记录当前位置,作为此次循环的起始位置
                ui_loop_flag = 0;
            }
            if (period_cnt % UI_list_pos->member.func.period == 0)
            {
                UI_list_pos->member.func.func(0); //执行函数,信号量在函数内部释放

                if (UI_list_pos->member.func.run_times > 0) //只执行有限次的函数
                {
                    UI_list_pos->member.func.run_times--;
                    if (UI_list_pos->member.func.run_times == 0)
                        UI_List_Delete(UI_list_pos);
                }
            }
        }
        period_cnt++;
    }
}

static void task_50ms_IRQHandler(void *parameter)
{
    rt_sem_release(&custom_ui_50ms_sem);
}

/***
 * @name
 * @brief 	UI绘制线程,每16ms绘制一次,
 * @param
 * @retval
 * @author dxy
 ***/
void CustomUI_Transmit_Thread(void *parameter)
{
    while (1)
    {
        SWDG_FEED(SWDG_UI_ID);
        rt_sem_take(&custom_ui_50ms_sem, RT_WAITING_FOREVER);
        rt_mutex_take(&custom_ui_mutex, RT_WAITING_FOREVER);
        Ref_Robot_ID();
        Get_UI_Data();
        ui_referee_intercom_tranamit(REF_ROBO_ID, REF_CLIENT_ID); //向裁判系统发送数据
        ui_loop_flag = 1;
        rt_mutex_release(&custom_ui_mutex);
        rt_sem_release(&custom_ui_sem);
    }
}

/**
 *@brief		UI链表初始化
 */
void UI_Func_List_Init(void)
{
    UI_Clear_Init_Flag();
    if (UI_List_Empty(&UI_list) == RT_FALSE)
        UI_list_Destroy(&UI_list);
    UI_List_Init(&UI_list);

    UI_Func_List_Add(&UI_list, Clear_Screen, 1, 1); //清屏
                                                    // UI_Func_List_Add(&UI_list, Fill_Full_Screen); //充满屏幕（测试用
                                                    /* 超级电容 */
    UI_Func_List_Add(&UI_list, Draw_SC_Bar_dyn, 0, 3);
    UI_Func_List_Add(&UI_list, Draw_SC_Bar_bkgd, 1, 1);
    /* 显示子弹模式 */
    UI_Func_List_Add(&UI_list, Draw_Strike_Mode, 0, 1);
    /* 绘制辅助瞄准线 */
    UI_Func_List_Add(&UI_list, Draw_AuxAim_Ballistic_Line, 0, 3);
    /* 绘制车身姿态 */
    UI_Func_List_Add(&UI_list, Draw_Attitude_Angle_dyn, 0, 2);
    UI_Func_List_Add(&UI_list, Draw_Attitude_Angle_bkgd, 1, 1);

    /* 底盘运动状态*/
    UI_Func_List_Add(&UI_list, Draw_Motion_Mode_dyn, 0, 1);
    UI_Func_List_Add(&UI_list, Draw_Motion_Mode_bkgd, 1, 1); //只执行一次

    /* 监视器状态 */
    UI_Func_List_Add(&UI_list, Draw_Monitor_dyn, 0, 1);
    UI_Func_List_Add(&UI_list, Draw_Monitor_bkgd, 1, 1);

    // // TODO
    // /* 受击提示 */
    // UI_Func_List_Add(&UI_list, Draw_Enemy_Orientation_dyn, 0, 3);

    // // TODO
    // /* 建筑受击提示 */
    // UI_Func_List_Add(&UI_list, Draw_Building_Attacked_dyn, 0, 1);

    /* 热量上限 */
    UI_Func_List_Add(&UI_list, Draw_HeatLimit_dyn, 0, 1);
    UI_Func_List_Add(&UI_list, Draw_HeatLimit_bkgd, 1, 1); //只执行一次
    /* 自瞄模式 */
    UI_Func_List_Add(&UI_list, Draw_Aimbot_Mode_dyn, 0, 1);
    UI_Func_List_Add(&UI_list, Draw_Aimbot_Mode_bkgd, 1, 1); //只执行一次

#ifndef CORE_USING_HERO
    UI_Func_List_Add(&UI_list, Draw_Magazine_dyn, 0, 1);
    UI_Func_List_Add(&UI_list, Draw_Magazine_bkgd, 1, 1); //英雄妹有弹仓盖
#endif
#ifdef CORE_USING_HERO
    UI_Func_List_Add(&UI_list, Draw_HERO_Cream_Help_bkgd, 0, 1); //英雄辅助瞄准
                                                                 //		UI_Func_List_Add(&UI_list, Draw_Height_dyn, 0,1);
                                                                 //		UI_Func_List_Add(&UI_list, Draw_Height_bkgd, 1,1); //只执行一次

    UI_Func_List_Add(&UI_list, Draw_PITCH_dyn, 0, 3);
    UI_Func_List_Add(&UI_list, Draw_PITCH_bkgd, 1, 1); //只执行一次

#endif
    UI_Func_List_Add(&UI_list, Draw_option, 0, 1); // 绘制二维按键选项卡
    /*************************暂时不用的部分********************************/
    // UI_Func_List_Add(&UI_list, Draw_LaneLine_bkgd, 1,1);
    /* 显示弹速 （暂时不用）*/
    // UI_Func_List_Add(&UI_list, Draw_Bullet_Speed_dyn,0);
    // UI_Func_List_Add(&UI_list, Draw_Bullet_Speed_bkgd, 1);
    // UI_Func_List_Add(&UI_list, Draw_Bullet_Speed_option, 0);
    /* 显示弹量 （暂时不用）*/
    // UI_Func_List_Add(&UI_list, Draw_Remain_Ammo_dyn,0);
    // UI_Func_List_Add(&UI_list, Draw_Remain_Ammo_bkgd, 1); //只执行一次
    /************************************************************************/
    UI_Func_List_Add(&UI_list, UI_Draw_END, 0, 1);
}

/***
 * @name
 * @brief	UI绘制线程初始化
 * @param
 * @retval
 * @author
 ***/
rt_err_t UI_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_thread_t thread;

    SWDG_START(SWDG_UI_ID);

    UI_Func_List_Init();

    res = rt_sem_init(&custom_ui_50ms_sem, "50ms_sem", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;
    res = rt_sem_init(&custom_ui_sem, "ui_sem", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;
    res = rt_mutex_init(&custom_ui_mutex, "dmutex", RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;

    thread = rt_thread_create("UI Draw",
                              CustomUI_Transmit_Thread,
                              RT_NULL,
                              THREAD_STACK_UI,
                              THREAD_PRIO_UI,
                              THREAD_TICK_UI);
    if (thread != RT_NULL)
    {
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
    {
        return RT_ERROR;
    }

    thread = rt_thread_create("UI Draw",
                              CustomUI_Update_Thread,
                              RT_NULL,
                              THREAD_STACK_UI,
                              THREAD_PRIO_UI,
                              THREAD_TICK_UI);
    if (thread != RT_NULL)
    {
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
    {
        return RT_ERROR;
    }

    //创建线程定时器
    rt_timer_init(&task_50ms,
                  "50ms_task",
                  task_50ms_IRQHandler,
                  RT_NULL,
                  50,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    //启动定时器
    res = rt_timer_start(&task_50ms);
    if (res != RT_EOK)
        return res;

    (void)UI_Data; // 防止编译器报变量未使用的 warning
    return RT_EOK;
}
