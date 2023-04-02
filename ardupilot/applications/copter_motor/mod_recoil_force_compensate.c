#include "mod_recoil_force_compensate.h"
#include "drv_dataserve.h"
#include "roboselect.h"

struct rt_semaphore gun_rec_sem; /* 用于接收信息的信号量 */
static gun_data copter_gun;
static recoil_data copter_recoil;
static rt_int8_t Package_ID;

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
    while (rt_sem_trytake(&gun_rec_sem) == RT_EOK)
        continue; //取完多余的信号量
    rt_sem_release(&gun_rec_sem);//接收到云台数据
}

void recoil_compensate_calculate(gun_data *gun , recoil_data *recoil_force)
{
    recoil_force->fresh_time = gun->fresh_time;
    recoil_force->bullet_num = BULLET_INITAL_NUM - gun->quantity;//通过初始弹量和已发射单弹量估计当前剩余弹量

    float f = (MASS + recoil_force->bullet_num*BULLET_MASS)*g;//通过当前自重估计升力

    float F_recoil = cosf(gun->Pitch.dji.angle)*gun->frequency*gun->speed*BULLET_MASS;//计算平均后坐力
    recoil_force->pitch_angle = F_recoil*cosf(gun->Yaw.dji.angle)/f/360*2*3.1415926f;//pitch轴角度补偿
    recoil_force->roll_angle = -F_recoil*sinf(gun->Yaw.dji.angle)/f/360*2*3.1415926f;//roll轴角度补偿

    float torque = 0.8f*F_recoil*GIMBAL_DIS;//计算平均总后坐力力矩。0.8为参量，根据实际进行调节
    recoil_force->x_torque = sinf(gun->Yaw.dji.angle)*torque;//计算补偿力矩
    recoil_force->y_torque = -cosf(gun->Yaw.dji.angle)*torque;
		
		if(COMPENSATE_OPEN)
			recoil_force->en_flag = gun->shooting_flag;
		else recoil_force->en_flag = 0;
};

void recoil_force_thread(void * param)
{
    copter_recoil.bullet_num = BULLET_INITAL_NUM;
    copter_recoil.en_flag = 0;
    while(1)
    {
        rt_sem_take(&gun_rec_sem,RT_WAITING_FOREVER);

        recoil_compensate_calculate(&copter_gun , &copter_recoil);//计算后坐力补偿

        /*数据服务器写入*/
        recoil_data *p =  Package_Pionter_Single(Package_ID,recoil_data);
        *p = copter_recoil;
        Package_Write_Pionter_End(Package_ID,recoil_data);
    }
}


void recoil_compensate_init(void)
{
    Package_Pionter_Add("compensate", recoil_data);
		Package_ID = Package_Find_Num("compensate");
    rt_sem_init(&gun_rec_sem, "gun_sem", 0, RT_IPC_FLAG_FIFO);

}
