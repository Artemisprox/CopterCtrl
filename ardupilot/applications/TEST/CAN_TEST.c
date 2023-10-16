#include "drv_canthread.h"
#include "can_receive.h"
#include "CAN_TEST.h"
#include <rtthread.h>
#include <board.h>
#include "pid.h"

pid_t Launch_spe;
static struct rt_semaphore Launch_2ms_sem; /* 用于接收消息的信号量 */
static struct rt_timer StrikeMotor_Tim;         /* 闭环线程定时器 */
Motor_t m_launch = {0};

Motor_t *Read_Gun_Motor(void)
{
    return &m_launch;
}

static void StrikeMotor_2ms_IRQHandler(void *parameter)
{
    while (rt_sem_trytake(&Launch_2ms_sem) == RT_EOK)
        continue; // 清空多余的信号量
    rt_sem_release(&Launch_2ms_sem);
}

static void TEST_CAN_Ctrl(void *parameter)
{
    struct rt_can_msg can_msg = {0};
    rt_int32_t count = 0;

    can_msg.id = 0x200;              /* ID 为 0x78 */
    can_msg.ide = RT_CAN_STDID;     /* 标准格式 */
    can_msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    can_msg.len = 8;                /* 数据长度为 8 */
    while(1)
    {
        float error;
        error = 6480 - m_launch.dji.speed ; 
        PID_Calculate(&m_launch.spe , error);
        /* 待发送的 8 字节数据 */
        can_msg.data[0] = (rt_uint8_t)((rt_int16_t)m_launch.spe.out >> 8);
        can_msg.data[1] = (rt_uint8_t)((rt_int16_t)m_launch.spe.out );
        can_msg.data[2] = 0x00;
        can_msg.data[3] = 0x00;
        can_msg.data[4] = 0x00;
        can_msg.data[5] = 0x00;
        can_msg.data[6] = 0x00;
        can_msg.data[7] = 0x00;

        rt_device_write(can1_dev, 0, &can_msg, sizeof(can_msg));
        rt_sem_take(&Launch_2ms_sem , RT_WAITING_FOREVER);
    }
}

static rt_err_t TEST_CANCtrl_init(void)
{
    rt_thread_t thread;
    rt_sem_init(&Launch_2ms_sem, "StrM_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("TEST_CAN_Ctrl", TEST_CAN_Ctrl, RT_NULL, 2048, 5, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
    rt_timer_init(&StrikeMotor_Tim, "Strk_Tim", StrikeMotor_2ms_IRQHandler, RT_NULL, 2,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 启动定时器 */
    rt_timer_start(&StrikeMotor_Tim);

    return RT_EOK;
}

rt_err_t CAN_Init(void)
{
	motor_init(&m_launch, 0X201, 36.f, ANGLE_CTRL_FULL, 8192, 360, 0, 0);
    pid_init(&m_launch.spe , 65, 20, 0, 4000, 9000, -9000);
    can1_init();
    can2_init();
    TEST_CANCtrl_init();
    return RT_EOK;
}
