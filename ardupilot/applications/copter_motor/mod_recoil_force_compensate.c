#include "mod_recoil_force_compensate.h"
#include "drv_dataserve.h"

struct rt_semaphore gun_rec_sem; /* 用于接收信息的信号量 */
gun_data copter_gun;

/**
 * @brief  读取can中的发射机构数据
 * @param  rxmsg：反馈报文数据
 * @retval None
 */
void gun_readmsg(rt_uint8_t rxmsg[])
{
    copter_gun.shooting_flag = rxmsg[0];
    copter_gun.speed = rxmsg[1];
    copter_gun.frequency = rxmsg[2];
    copter_gun.quantity = (uint16_t) ( rxmsg[4] << 8 | rxmsg[3] );
    copter_gun.fresh_time = rt_tick_get();

    rt_sem_release(&gun_rec_sem);//接收到云台数据
}

void recoil_compensate_calculate(gun_data *gun , Motor_t *yaw , Motor_t *pitch)
{
    
};
