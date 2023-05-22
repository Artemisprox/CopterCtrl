#include "func_uart_rec.h"
#include "drv_thread.h"
#include <board.h>

//光流数据接收缓存区
uint8_t Rec_DataBuff[3][20];
// 接收错误计数
uint16_t Rec_err_count[3];

/* 串口设备句柄 */
static rt_device_t Flow_front_UartDev;
static rt_device_t Flow_behind_UartDev;

//static rt_sem_t flow_front_sem;
//static rt_sem_t flow_behind_sem;

/*光流数据*/
upxiels_rawdata flow_front_data;
upxiels_rawdata flow_behind_data;

static char front_msg_pool[256];
static char behind_msg_pool[256];

/* 消息队列控制块 */
static struct rt_messagequeue flow_front_msg;
static struct rt_messagequeue flow_behind_msg;

/* 串口接收消息结构*/
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

// 串口接收中断回调函数
static rt_err_t flowfront_RecCallback(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;
    result = rt_mq_send(&flow_front_msg, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}
static rt_err_t flowbehind_RecCallback(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;
    result = rt_mq_send(&flow_behind_msg, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}


/**
 * @brief 错误计数
 * @param encoder_num
 */
static void IncErrCount(flow_num flow_num)
{
    switch (flow_num)
    {
    case flow_front:
        Rec_err_count[0]++;
        break;
    case flow_behind:
        Rec_err_count[1]++;
        break;
		default:
			break;
    }
}

/**
 * @brief 错误计数清零
 * @param encoder_num
 */
static void IncErrCountClear(flow_num flow_num)
{
    switch (flow_num)
    {
    case flow_front:
        Rec_err_count[0] = 0;
        break;
    case flow_behind:
        Rec_err_count[1] = 0;
        break;
		default:
				break;
    }
}

/**
 * @brief
 * @param encoder_num
 */
void *FlowData_Get(flow_num flow_num)
{
    switch (flow_num)
    {
    case flow_front:
        return (void *)&flow_front_data;
    case flow_behind:
        return (void *)&flow_behind_data;
    default:
        return RT_NULL;
    }
}

rt_uint32_t i = 0;
//光流数据处理线程
static void flow_front_thread(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
//    rt_int8_t LastData_Valid = 0;
    //初始化
    
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&flow_front_msg, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, Rec_DataBuff[0], msg.size);
            if (rx_length != 14)
            { 
								i = rx_length;
							// 如果长度不对，则直接跳过，但是必须从rt_device_read读出，否则缓冲区会溢出
                continue;
            }
            Rec_DataBuff[0][rx_length] = '\0';

            UP_Flow_Process(Rec_DataBuff[0],Data_len,FlowData_Get(flow_front));
            //TF_mini_data.Data_fresh_time = rt_tick_get(); // 刷新数据的更新时间
						
        }
    }
}

static void flow_behind_thread(void  *parameter)
{
     struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
//    rt_int8_t LastData_Valid = 0;
    //初始化
    
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&flow_behind_msg, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, Rec_DataBuff[1], msg.size);
            if (rx_length != 14)
            { // 如果长度不对，则直接跳过，但是必须从rt_device_read读出，否则缓冲区会溢出
                continue;
            }
            Rec_DataBuff[1][rx_length] = '\0';

			UP_Flow_Process(Rec_DataBuff[1],Data_len,FlowData_Get(flow_behind));
        }
    }
}

struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
// 串口接收数据初始化
rt_err_t UART_REC_Init(void)
{
    /* 初始化消息队列 */
    rt_mq_init(&flow_front_msg, "front_rx_mq",
               front_msg_pool,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(front_msg_pool),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 初始化消息队列 */
    rt_mq_init(&flow_behind_msg, "behind_rx_mq",
               behind_msg_pool,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(behind_msg_pool),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */


    rt_err_t res;
    rt_thread_t thread;
	
    // 查找串口设备
    Flow_front_UartDev = rt_device_find(Flow_front_DevName);
    if (Flow_front_UartDev == RT_NULL)
    {
        while (1)
            continue;
    }
	
		/* step2：修改串口配置参数 */
		config.baud_rate = BAUD_RATE_19200;        //修改波特率为 19200
		config.data_bits = DATA_BITS_8;           //数据位 8
		config.stop_bits = STOP_BITS_1;           //停止位 1
		config.bufsz     = 128;                   //修改缓冲区 buff size 为 128
		config.parity    = PARITY_NONE;           //无奇偶校验位
		
		Flow_behind_UartDev = rt_device_find(Flow_behind_DevName);
    if (Flow_behind_UartDev == RT_NULL)
    {
        while (1)
            continue;
    }
		rt_device_control(Flow_behind_UartDev, RT_DEVICE_CTRL_CONFIG, &config); 
    res = rt_device_open(Flow_behind_UartDev, RT_DEVICE_FLAG_DMA_RX);
    if (res != RT_EOK)
    {
        while (1)
            continue;
    }
	
		/* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
		rt_device_control(Flow_front_UartDev, RT_DEVICE_CTRL_CONFIG, &config);
		// 打开串口设备
    res = rt_device_open(Flow_front_UartDev, RT_DEVICE_FLAG_DMA_RX);
    if (res != RT_EOK)
    {
        while (1)
            continue;
    }
    
    thread = rt_thread_create("flow_front", flow_front_thread,
                              RT_NULL, 1024,
                              THREAD_PRIO_FLOW_RX, 1);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return RT_ERROR;
    }
    thread = rt_thread_create("flow_behind", flow_behind_thread,
                              RT_NULL, 1024,
                              THREAD_PRIO_FLOW_RX, 1);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return RT_ERROR;
    }
    
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(Flow_front_UartDev, flowfront_RecCallback);
    rt_device_set_rx_indicate(Flow_behind_UartDev, flowbehind_RecCallback);

    return RT_EOK;
}
