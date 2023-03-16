#include "func_state.h"
#include "drv_thread.h"
#include "func_sensor.h"
#include "func_remote.h"
#include "drv_IMU.h"
#include "drv_dataserve.h"
#include "drv_battery.h"
#include "mod_error_handle.h"

remote_data copter_remote;
pos_sensor copter_pos;
IMU_t copter_atti;
battery copter_power;

data_check copter_data_valid = {0};
status copter_status = {0};

struct rt_semaphore State_20ms_sem; /* ���ڽ�����Ϣ���ź��� */
static struct rt_timer State_decide_tim;/* �ջ��̶߳�ʱ�� */


static void State_decide_20ms_IRQHandler(void *parameter)
{
     while (rt_sem_trytake(&State_20ms_sem) == RT_EOK)
        continue; //取完多余的信号量
    rt_sem_release(&State_20ms_sem);
}

//遥控器状态查询
static uint8_t rc_status_check(remote_data *data)
{
    uint32_t now_tick = rt_tick_get();
    if(now_tick - data->fresh_time >= 100000 )//100ms未接收到遥控器数据判断为离线
    {
        copter_data_valid.rc_valid = 0;
        copter_status.rc_status = 0;
    }
    else
    {
        copter_data_valid.rc_valid = 0;
        copter_status.rc_status = 0;
    }

    if(now_tick - data->fresh_time >= 2000000 )//2s后仍未恢复通信飞机断电
        copter_status.emergency =1;

    return copter_data_valid.rc_valid;
}

//起飞前状态检查
static uint8_t arm_confirm(data_check *data_valid , remote_data data)
{
    uint8_t flag = 0;
    if(data.switch_arm == 0 && data.switch_arm_change)
    {
        //切换至起飞模式
        //检查摇杆是否回中
        if(data.pitch_middle_flag && data.roll_middle_flag && data.throttle_low_flag && data.yaw_middle_flag)
        {
            //检查各个数据是否齐全
            if( data_valid->atti_valid && data_valid->battery_OK && data_valid->rc_valid )
                {
                    copter_status.flight_status = ARMED;
                    flag = 1;
                }

        }else
        {
            /*摇杆未回中*/
            error_write(THROTTLE_HIGH);
        }
    }
    return flag;
}

//升空监测
static uint8_t flying_check(void)
{
    uint8_t flag = 0;
    if(copter_status.flight_status == 0 && copter_remote.throttle_low_flag == 1 )
    {
        copter_status.flight_status = FLYING;
        flag = 1;
    }    
    
    return flag;
}

//降落检测
static uint8_t land_check(void)
{
    static int time_tick = 0;
    uint8_t flag = 0;
    if(copter_remote.throttle_low_flag == 1)
    {
        time_tick ++;
    }

    if(time_tick >= 100)
    {
        copter_status.flight_status = LAND;
        time_tick = 0;
        flag = 1;
    }

    return flag;
}

//飞行模式切换
uint8_t mode_change(data_check *data_valid , remote_data data)
{
    switch (data.switch_mode)
    {
    case POSITION:
        if( data_valid->pos_valid )
        {
            copter_status.mode = POSITION;
            break;
        }
    case HEIGHT:
        if( data_valid->height_valid )
        {
            copter_status.mode = HEIGHT;
            break;
        }
    default:
            copter_status.mode = STABILIZATION;
            break;
    }
    
}

//状态决策线程
static void State_decide_thread_entry(void *parameter)
{
   
    while(1)
    {
        /*数据服务器更新*/

        
        /*数据源检测*/
        if(copter_remote.switch_arm == EMERGENCY_STOP)
            copter_status.emergency = 1;
        else copter_status.emergency = 0;

        if(!(copter_data_valid.battery_OK = copter_power.Battery_data_rec && copter_power.Battery_status))
            error_write(CHECK_BATTERY);
        if(!(copter_data_valid.pos_valid = copter_pos.pos_valid))
            error_write(POS_LOST);
        if(!(copter_data_valid.height_valid = copter_pos.height_valid))
            error_write(HEIGHT_LOST);
        if(rc_status_check(&copter_remote))
        {
            error_write(RC_LOST);
        }
        if(!(copter_data_valid.atti_valid = copter_atti.atti_ready))
        {
            error_write(ATTI_LOST); 
            copter_status.emergency = 1;
        }
        
        
        mode_change(&copter_data_valid,copter_remote);

        if(!copter_status.emergency)
        {
            if(copter_status.flight_status == READY)
                arm_confirm(&copter_data_valid , copter_remote);//起飞检测
            else if(copter_status.flight_status == ARMED)
                flying_check();//升空检测
            else if(copter_status.flight_status == FLYING)
                land_check();//降落检查
        }else
        {
            copter_status.flight_status = READY;//每次紧急停止都取消起飞，防止解除紧急停止后起桨叶
        }

        error_read();
    }

}

void copter_state_init(void)
{
    copter_status.emergency = 0;
    copter_status.flight_status = READY;
    copter_status.mode = STABILIZATION;
    copter_status.recoil_compensate_enable = 0;
}

rt_err_t StateDecide_Init(void)
{
    //初始状态设置
    copter_status_init();
    //错误处理
    error_handle_init();
	/*决策线程*/
    rt_thread_t thread;
    rt_sem_init(&State_20ms_sem, "Position_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("Pos_message", State_decide_thread_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器线程*/
    rt_timer_init(&State_decide_tim, "State_decide_tim", State_decide_20ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 定时器开始 */
    rt_timer_start(&State_decide_tim);

   return RT_EOK;
}
