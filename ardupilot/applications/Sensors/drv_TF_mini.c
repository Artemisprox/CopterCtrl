#include "drv_TF_mini.h"
#include <rtdevice.h>
#include "board.h"
#include "drv_thread.h"

static rt_device_t serial = RT_NULL;
struct rt_semaphore TF_mini_sem; /* 用于接收消息的信号量 */
uint8_t TF_mini_rx_buffer[RT_SERIAL_RB_BUFSZ + 1];

static char msg_pool[256];

/* 串口设备句柄 */
static rt_device_t serial;
/* 消息队列控制块 */
static struct rt_messagequeue TF_mini_rx_mq;

/*TFmini接收数据结构*/
TF_mini_rec TF_mini_data = {0};

/* 串口接收消息结构*/
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};


/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;
    result = rt_mq_send(&TF_mini_rx_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}

void TF_mini_DataProcess(uint8_t *pData)
{
//    static uint8_t first_flag = 1 ;
    int i = 0;
    uint8_t byte1 = 0;
    uint8_t byte2 = 0;
		uint8_t check = 0;
		static uint16_t distance , distance_last;
//		static uint32_t last_time = 0;
		long sum = 0;
    if (pData == NULL)
    {
        return;
    }

		byte1 = pData[0];//起始帧
		byte2 = pData[1];//起始帧
		check = pData[8];//校验帧
		
		for(i = 0; i < 8 ; i++ )
		{
			sum += pData[i];
		}
		
		if(check != (uint8_t) (sum & 0xFF) )
			return;
		
		if(byte1 == 0x59 && byte2 == 0x59 )
		{
			
			distance = ((uint16_t)pData[2] | ((uint16_t)pData[3] << 8));
			if(distance == 0xFFFF || distance == 0xFFFE || distance == 0xFFFD )//数据获取错误
			{
				TF_mini_data.Data_valid = 0;
				return;
			}else
			{
			TF_mini_data.distance = distance/100.0f;
			TF_mini_data.strength = ((uint16_t)pData[4] | ((uint16_t)pData[5] << 8));
			TF_mini_data.temperature = ((uint16_t)pData[6] | ((uint16_t)pData[7] << 8));
			TF_mini_data.data_num ++;
      TF_mini_data.Data_fresh_time = rt_tick_get();
      //if(first_flag == 0)
			//	TF_mini_data.distance_v = (distance - distance_last)/2.0f;//使用高度变化量估计速度
      //else 
      //  first_flag = 0;
			TF_mini_data.Data_valid = 1;
      distance_last = distance;
			}
		}
    
    
}



static void TF_serial_thread_entry(void *parameter)
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
        result = rt_mq_recv(&TF_mini_rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, TF_mini_rx_buffer, msg.size);
						//rt_device_read(serial, -1, TF_mini_buffer_p, 1);
						//TF_mini_buffer_p ++ ;
            if (rx_length != 9)
            { // 如果长度不对，则直接跳过，但是必须从rt_device_read读出，否则缓冲区会溢出
                continue;
            }
            TF_mini_rx_buffer[rx_length] = '\0';

            TF_mini_DataProcess(TF_mini_rx_buffer);

            //TF_mini_data.Data_fresh_time = rt_tick_get(); // 刷新数据的更新时间
						
						
		 //rt_memcpy(&RC_data_last, &RC_data, sizeof(RC_data));
     //释放遥控器数据处理信号量
     //   while (rt_sem_trytake(&TF_mini_sem) == RT_EOK)
     //       continue;
     //   rt_sem_release(&TF_mini_sem);
    }
}
}


rt_err_t TF_mini_Init(void)
{
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(TF_mini_device);

    /* step2：修改串口配置参数 */
    config.baud_rate = BAUD_RATE_115200;      //修改波特率为 9600
    config.data_bits = DATA_BITS_8; //数据位 8
    config.stop_bits = STOP_BITS_1; //停止位 1
    config.bufsz = 128;             //修改缓冲区 buff size 为 128
    config.parity = PARITY_EVEN;    //

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* 初始化消息队列 */
    rt_mq_init(&TF_mini_rx_mq, "rx_mq",
               msg_pool,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX );
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);

    //初始化信号量
    rt_sem_init(&TF_mini_sem, "TF_mini_rec", 0, RT_IPC_FLAG_FIFO);

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", TF_serial_thread_entry, RT_NULL, 2048, THREAD_PRIO_SENSOR_UART_RX, 5);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
   
   return RT_EOK;
}
