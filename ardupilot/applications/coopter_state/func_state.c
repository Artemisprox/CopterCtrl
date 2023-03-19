#include "func_state.h"
#include "drv_thread.h"
#include "func_sensor.h"
#include "func_remote.h"
#include "drv_IMU.h"
#include "drv_dataserve.h"
#include "drv_battery.h"
#include "mod_error_handle.h"
#include "mod_recoil_force_compensate.h"

remote_data copter_remote;
pos_sensor copter_pos;
IMU_t copter_atti;
battery copter_power;
gun_data copter_gun;

data_check copter_data_valid = {0};
status copter_status = {0};

struct rt_semaphore State_20ms_sem; /* 定时信号量 */
static struct rt_timer State_decide_tim;/* 定时器 */
static rt_int8_t remote_ID,IMU_ID,battery_ID,sensor_ID,gun_ID,status_ID;

static void State_decide_20ms_IRQHandler(void *parameter)
{
     while (rt_sem_trytake(&State_20ms_sem) == RT_EOK)
        continue; //取完多余的信号量
    rt_sem_release(&State_20ms_sem);
}

//云台状态查询
static uint8_t gimbal_status_check(gun_data *data)
{
    uint32_t now_tick = rt_tick_get();
    if(now_tick - data->fresh_time >= 200000 )//200ms未接收到云台数据判断为离线
    {
        copter_data_valid.gimbal_OK = 0;
        copter_status.recoil_compensate_enable = 0;
    }
    else
    {
        copter_data_valid.gimbal_OK = 1;
        copter_status.recoil_compensate_enable = 1;
    }

    return copter_status.recoil_compensate_enable;
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
    }else
    {
       time_tick = 0; 
    }

    if(time_tick >= LAND_DELAG_TIME/20 )
    {
        copter_status.flight_status = READY;
        copter_status.emergency = 1;
        time_tick = 0;
        flag = 1;
    }

    return flag;
}

//提出飞行模式切换申请
static void mode_change(data_check *data_valid , remote_data data)
{
    static uint8_t first_flag = 1;
    if(first_flag == 1)
    {
        copter_status.mode =  data.switch_mode;
        first_flag = 0;
    }else if (first_flag == 0 && data.switch_mode_change)
    {
        copter_status.mode =  data.switch_mode;
    }   
}

//检查飞行模式是否能够执行
static void mode_check(data_check *data_valid)
{
    switch (copter_status.mode)
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

static void package_update(void)
{
    remote_data *p_1 =  Package_Pionter_Single(remote_ID,remote_data);
	copter_remote = *p_1 ;
	Package_Write_Pionter_End(remote_ID,remote_data);

    IMU_t *p_2 =  Package_Pionter_Single(IMU_ID,IMU_t);
	copter_atti = *p_2 ;
	Package_Write_Pionter_End(IMU_ID,IMU_t);

    pos_sensor *p_3 =  Package_Pionter_Single(sensor_ID,pos_sensor);
	copter_pos = *p_3 ;
	Package_Write_Pionter_End(sensor_ID,pos_sensor);

    battery *p_4 =  Package_Pionter_Single(battery_ID,battery);
	copter_power = *p_4 ;
	Package_Write_Pionter_End(battery_ID,battery);

    gun_data *p_5 =  Package_Pionter_Single(gun_ID,gun_data);
	copter_gun = *p_5 ;
	Package_Write_Pionter_End(gun_ID,gun_data);

}

//状态决策线程
static void State_decide_thread_entry(void *parameter)
{
   
    while(1)
    {
        /*数据服务器更新*/
        package_update();
        /*数据源检测*/
        if(copter_remote.switch_arm == EMERGENCY_STOP_T)
            copter_status.emergency = 1;
        else copter_status.emergency = 0;

        if(gimbal_status_check(&copter_gun))
        {
            error_write(GIMBAL_LOST);
        }
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
        
        //飞行模式
        mode_change(&copter_data_valid,copter_remote);
        mode_check(&copter_data_valid);

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

        /*数据服务器写入*/
        status *p =  Package_Pionter_Single(status_ID,status);
        *p = copter_status;
        Package_Write_Pionter_End(status_ID,status);
        
        error_read();

    }

}

static void copter_status_init(void)
{
    copter_status.emergency = 0;
    copter_status.flight_status = READY;
    copter_status.mode = STABILIZATION;
    copter_status.recoil_compensate_enable = 0;
    Package_Pionter_Add("status", status);
		status_ID = Package_Find_Num("status");
}

rt_err_t StateDecide_Init(void)
{  
    /*数据服务器ID查找*/
    remote_ID = Package_Find_Num("remote");
    IMU_ID = Package_Find_Num("IMU");
    battery_ID = Package_Find_Num("battery");
    sensor_ID = Package_Find_Num("sensor_ID");
    gun_ID = Package_Find_Num("compensate");
    //初始状态设置
    copter_status_init();
    //错误处理
    error_handle_init();
	/*决策线程*/
    rt_thread_t thread;
    rt_sem_init(&State_20ms_sem, "State_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("State_message", State_decide_thread_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器线程*/
    rt_timer_init(&State_decide_tim, "State_decide_tim", State_decide_20ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 定时器开始 */
    rt_timer_start(&State_decide_tim);

   return RT_EOK;
}
