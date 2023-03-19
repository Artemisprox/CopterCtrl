#include "mod_recoil_force_compensate.h"
#include "drv_dataserve.h"

struct rt_semaphore gun_rec_sem; /* 用于接收信息的信号量 */
gun_data copter_gun;
static uint8_t Package_ID;

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
    copter_gun.Pitch.dji.angle = (rt_int32_t) (rxmsg[8] << 8*3 | rxmsg[7] << 8*2 | rxmsg[6] << 8 | rxmsg[5] );
    copter_gun.Yaw.dji.angle = (rt_int32_t) (rxmsg[12] << 8*3 | rxmsg[11] << 8*2 | rxmsg[10] << 8 | rxmsg[9] );
    copter_gun.fresh_time = rt_tick_get();

    rt_sem_release(&gun_rec_sem);//接收到云台数据
}

void recoil_compensate_calculate(gun_data *gun , Motor_t *yaw , Motor_t *pitch)
{
    /*数据服务器写入*/
	gun_data *p =  Package_Pionter_Single(Package_ID,gun_data);
	*p = copter_gun;
	Package_Write_Pionter_End(Package_ID,gun_data);

};


void recoil_compensate_init(void)
{

    Package_Pionter_Add("compensate", copter_gun);
	Package_ID = Package_Find_Num("compensate");
    rt_sem_init(&gun_rec_sem, "gun_sem", 0, RT_IPC_FLAG_FIFO);
}
