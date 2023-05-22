#include "app_Data_Send.h"
#include "drv_thread.h"
#include "drv_Gpio_Ctrl.h"
#include "func_uart_rec.h"
#include <board.h>

static rt_timer_t timer1;

static rt_sem_t Send_sem;
static rt_device_t Send_dev;
// 串口发送数据
UART_Send_Data SendData_buff;

// 发送数据错误计数
rt_uint16_t send_err_count;

static upxiels_rawdata flow_send; 

/**
 * @brief 自定义控制器数据发送
 * @param data
 */
static void Data_Write(upxiels_rawdata *in,
                                 UART_Send_Data *data_out)
{
    data_out->head[0] = 0xFE;
    data_out->head[1] = 0x0A;
    data_out->flow_x_integral[0] = (rt_int16_t)in->flow_x_integral;
    data_out->flow_y_integral[0] = (rt_int16_t)in->flow_y_integral;
    data_out->integration_timespan[0] = (rt_int16_t)in->integration_timespan;
		data_out->gound_distance[0] = (rt_int16_t)0xff;
		data_out->vaild = in->quality ;
		data_out->version = 0x01;
		data_out->end = 0x55;
			
		int i;
		for(i = 2 ; i < 12 ; i++)
			data_out->xor_check ^= *((uint8_t*)(data_out + i));
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
				FlowDataFusion((upxiels_rawdata*)FlowData_Get(flow_front) , FlowData_Get(flow_behind) , &flow_send );
				Data_Write(&flow_send , &SendData_buff);
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
                              THREAD_PRIO_FLOW_TX, 1);
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
