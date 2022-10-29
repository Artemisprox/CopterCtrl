#include "app_ChassisCtrl.h"
#include "func_ModeTrigger.h"
#include "HThread_data.h"
#include "drv_smooth.h"
#include "drv_wheel.h"
#include "mod_Monitor.h"

static Ctrl_schedule_t  ctrl_schedulce = {.ctrl_mode = PREEMPTIVE,.fix_source = CS_GIMBAL};
static Ctrl_source_t    ctrl_source[SOURCE_NUM];


/*线程定时器*/
static struct rt_semaphore 	ChassCtrl_sem;	
static void ChassCtrl_IRQHandler(void *parameter)
{
    rt_sem_release(&ChassCtrl_sem);
}


/**
* @brief    模式状态调度
* @param    Cctrl    Ctrldata_t指针
* @return   None
*/
static void ModeState_Scheduler(Ctrl_data_t Cctrl)
{    
    switch(Cctrl.motion_mode)
    {
        case CHASSIS_ONLY:  Cctrl.xyw = MotPack_Only_Chass(Cctrl.xyw);                                  break;
        case SMALL_TOP:     Cctrl.xyw = MotPack_Small_Top(Cctrl.xyw);                                   break;
        case FOLLOW_GIMBAL: Cctrl.xyw = MotPack_Follow_Gim(Cctrl.xyw,Cctrl.fol_angle);                  break;
        case OFFSET_ONLY:   Cctrl.xyw = MotPack_Offset_OnlyChass(Cctrl.xyw,Cctrl.pos);                  break;
        case OFFSET_TOP:    Cctrl.xyw = MotPack_Offset_SmallTop(Cctrl.xyw,Cctrl.pos);                   break;
        case OFFSET_FOLLOW: Cctrl.xyw = MotPack_Offset_FollowGim(Cctrl.xyw,Cctrl.pos,Cctrl.fol_angle);  break;
        case SPIN_XY:       Cctrl.xyw = MotPack_Spin_Dot(Cctrl.pos,Cctrl.dot_angvel);                   break;
        case CHASSIS_STOP:  Cctrl.xyw = MotPack_Only_Chass((Mot_base_t){0,0,0});                        break;
        case CHASSIS_NOCTRL:                                                                            break;
        default:            RT_ASSERT(0);                                                               break;
    }//switch

    ExMotMod_Output(Cctrl.xyw,Cctrl.motion_mode,Cctrl.fol_angle);
}

/**
* @brief    底盘控制
* @param    Cctrl    Ctrldata_t 控制数据指针
* @return   None
*/
static void Chassis_Ctrl(Ctrl_data_t Cctrl)
{
    ModeTrig_Scheduler(Cctrl.motion_mode);
    ModeState_Scheduler(Cctrl);
}


/**
 * @brief   底盘控制线程
 * @param   parameter
 * @return  None
 */
static void Chassis_Ctrl_Thread(void *parameter)
{
    SWDG_START(SWDG_CHASSISCTRL_ID);
    while (1)
    {
        rt_sem_take(&ChassCtrl_sem,RT_WAITING_FOREVER);
        switch (ctrl_schedulce.ctrl_mode)
        {
            case PREEMPTIVE:
                Chassis_Ctrl(ctrl_source[Source_Get_MaxPrio()].ctrl.data);
                break;    

            case FIXATION:
                Chassis_Ctrl(ctrl_source[ctrl_schedulce.fix_source].ctrl.data);
                break;    
        }
        SWDG_FEED(SWDG_CHASSISCTRL_ID);
    }
}


/**
 * @brief   数据源结构体初始化
 * @param   cs          Ctrl_source_t指针
 * @param   level       数据源优先级
 * @param   if_valid    数据源是否有效
 * @return  None
 */
static void Source_Struct_Init(Ctrl_source_t *cs,SerialNum_e level,rt_bool_t if_valid)
{
    cs->level = level;
    cs->ctrl.data = (Ctrl_data_t)CTRL_DATA_INIT_ZERO(SMALL_TOP);
    cs->if_valid = if_valid;
}


/**
 * @brief   底盘控制初始化
 * @param   None
 * @return  rt_err_t 是否正常初始化
 * @author  lfp
 */
rt_err_t Chassis_Ctrl_Init(void)
{
    rt_err_t res;
    rt_thread_t thread = RT_NULL;

    /*底盘控制的各个组件初始化*/
    res = Wheels_Init();        if ( res != RT_EOK) return res;
    res = Smooth_Init();        if ( res != RT_EOK) return res;
    
    /*初始化数据源结构体*/
    Source_Struct_Init(&ctrl_source[CS_GIMBAL] ,SE_GIMBAL_LEVEL ,SE_GIMBAL_IF_VALID);
    Source_Struct_Init(&ctrl_source[CS_TEST]   ,SE_TEST_LEVEL   ,SE_TEST_IF_VALID);
    Source_Struct_Init(&ctrl_source[CS_MONITOR],SE_MONITOR_LEVEL,SE_MONITOR_IF_VALID);

    //初始化信号量
    res = rt_sem_init(&ChassCtrl_sem, "Cctrl_sem", 0, RT_IPC_FLAG_FIFO);
    if ( res != RT_EOK)
		return res;

    //初始化底盘线程
    thread = rt_thread_create("Chassis_Thread",		    
                               Chassis_Ctrl_Thread,	    
                               RT_NULL,					
                               THREAD_STACK_CHASSIS_CTRL,
                               THREAD_PRIO_CHASSIS_CTRL,
                               THREAD_TICK_CHASSIS_CTRL);

    //线程创建失败返回false
    if(thread == RT_NULL)
        return RT_ERROR;

    //线程启动失败返回false
    if(rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;

    //创建线程定时器
    rt_timer_t timer = rt_timer_create("Cctrl_timer",
                       ChassCtrl_IRQHandler,
                       RT_NULL,
                       CHASSIS_CTRL_PERIOD,
                       RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    //启动定时器
    res = rt_timer_start(timer);
    if ( res != RT_EOK)
		return res;
	
    return RT_EOK;
}
////////////////////////////////////////////////////////向外接口函数////////////////////////////////////////////////////////
/**
 * @brief   获取数据源最大优先级下标
 * @param   None
 * @return  Ctrl_source_e
 */
Ctrl_source_e Source_Get_MaxPrio(void)
{
    int index = 0,temp_max = -1;

    //轮询取优先级最大的下标
    for (int cs = 0; cs < SOURCE_NUM; cs++)
    {
        //数据源无效，跳过
        if(ctrl_source[cs].if_valid == RT_FALSE)
            continue;
        //判断较大值
        if(ctrl_source[cs].level > temp_max)
        {
            temp_max = ctrl_source[cs].level;
            index = cs;
        }
    }

    return (Ctrl_source_e)index;
}


/**
 * @brief   设置数据源优先级
 * @note    尽量不改优先级
 * @param   cs      数据源下标
 * @param   level   优先级
 * @return  None
 */
void Source_Set_Priority(Ctrl_source_e cs,SerialNum_e level)
{
    ctrl_source[cs].level = level;
}


/**
 * @brief   设置数据源为最大优先级
 * @note    不考虑数据类型溢出，尽量不使用该函数
 * @param   cs      数据源下标
 * @return  None
 */
void Source_Set_MaxPrio(Ctrl_source_e cs)
{
    //设置为最大优先级+1
    ctrl_source[cs].level = ctrl_source[Source_Get_MaxPrio()].level + 1;
}


/**
 * @brief   数据源写入数据
 * @param   cs      数据源下标
 * @param   data    控制数据
 * @return  None
 */
void Source_Write_Data(Ctrl_source_e cs,Ctrl_data_info_t info)
{
    ctrl_source[cs].ctrl.info = info;
}


/**
 * @brief   设置数据源状态
 * @param   cs      数据源下标
 * @param   state   数据源启动or关闭
 * @return  None
 */
void Source_Set_State(Ctrl_source_e cs,rt_bool_t state)
{
    ctrl_source[cs].if_valid = state;
}


/**
 * @brief   获得控制模式
 * @param   None
 * @return  Schedule_mode_e
 */
Schedule_mode_e Schedule_Get_CtrlMode(void)
{
    return ctrl_schedulce.ctrl_mode;
}


/**
 * @brief   设置控制模式
 * @param   mode    模式
 * @return  None
 */
void Schedule_Set_CtrlMode(Schedule_mode_e mode)
{
    ctrl_schedulce.ctrl_mode = mode;
}


/**
 * @brief   获得固定数据源
 * @param   None
 * @return  Ctrl_source_e
 */
Ctrl_source_e Schedule_Get_FixSource(void)
{
    return ctrl_schedulce.fix_source;
}


/**
 * @brief   设置固定数据源
 * @param   source  数据源下标
 * @return  None
 */
void Schedule_Set_FixSource(Ctrl_source_e source)
{
    ctrl_schedulce.fix_source = source;
}

