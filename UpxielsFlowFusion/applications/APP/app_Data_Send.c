#include "app_Data_Send.h"
#include "drv_thread.h"
#include "drv_Encoder.h"
#include "drv_Resolve.h"
#include "drv_Gpio_Ctrl.h"
#include "drv_CustCtrler_Data.h"
#include <board.h>

static rt_timer_t timer1;

static rt_sem_t Send_sem;
static rt_device_t Send_dev;
// 串口发送数据
UART_Send_Data SendData_buff;
static VectorXYZ_float_Str XYZData_Rectemp = {0};
static VectorXYZ_float_Str XYZData_Resolvetemp = {0};

// 发送数据错误计数
rt_uint16_t send_err_count;

/**
 * @brief 自定义控制器数据发送
 * @param data
 */
static void CustCtrlerData_Write(VectorXYZ_float_Str *in,
                                 UART_Send_Data *data_out)
{

    data_out->head[0] = 0xFD;
    data_out->head[1] = 0xFE;

    data_out->move_X = (rt_int16_t)in->x;
    data_out->move_Y = (rt_int16_t)in->y;
    data_out->move_Z = (rt_int16_t)in->z;

    data_out->Sum = data_out->move_X +
                    data_out->move_Y +
                    data_out->move_Z;
	
}

static void timer1_callback(void *parameter)
{
    rt_sem_release(Send_sem);
}

// 数据发送线程
static void Send_thread(void *parameter)
{
    rt_uint8_t size;
    while (1)
    {
        // 定周期发送给三维鼠标
        rt_sem_take(Send_sem, RT_WAITING_FOREVER);

        // 从数据服务器取数据
        CustCtrler_Data_L_Read(&XYZData_Rectemp);
        // 数据解算
        RoboXYZ_Resolve(&XYZData_Rectemp,
                        &XYZData_Resolvetemp);
        // 矢量判零
        if (Float3D_ZeroIf(&XYZData_Resolvetemp,MOD_TOLERANCE))
        {
            GPIO_State_Set(GREEN_LIGHT, Open); // 绿灯开启
            GPIO_State_Set(RAD_LIGHT, Close);  // 红灯关闭
        }
        else
        {
            GPIO_State_Set(RAD_LIGHT, Open);    // 红灯打开
            GPIO_State_Set(GREEN_LIGHT, Close); // 绿灯关闭
        }
        // 数据写入
        CustCtrlerData_Write(&XYZData_Resolvetemp,
                             &SendData_buff);
        // 数据发送
        size = rt_device_write(Send_dev, 0, (rt_uint8_t *)&SendData_buff, SENDDATA_LEN);
        if (size != SENDDATA_LEN)
        {
            send_err_count++;
        }
    }
}

rt_err_t UART_Send_Init(void)
{
    rt_err_t res;
    rt_thread_t thread;

    Send_sem = rt_sem_create("Send_sem", 0, RT_IPC_FLAG_PRIO);

    // 查找串口设备
    Send_dev = rt_device_find(SendDev_name);
    if (Send_dev == RT_NULL)
    {
        while (1)
            continue;
    }
    // 打开串口设备
    res = rt_device_open(Send_dev, RT_DEVICE_FLAG_DMA_TX);
    if (res != RT_EOK)
    {
        while (1)
            continue;
    }
    thread = rt_thread_create("Send_sem", Send_thread,
                              RT_NULL, 1024,
                              THREAD_PRIO_MOVE_XYZ_TX, 1);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return RT_ERROR;
    }

    /* 创建定时器 1  周期定时器 */
    timer1 = rt_timer_create("timer1", timer1_callback,
                             RT_NULL, DATASEND_TIMER_PIRIOD,
                             RT_TIMER_FLAG_PERIODIC);
    /* 启动定时器 1 */
    if (timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }
    else
    {
        while (1)
            continue;
    }

    return RT_EOK;
}
