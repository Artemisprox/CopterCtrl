#include "drv_NimingFlow.h"
#include <rtdevice.h>
#include "board.h"
#include "drv_thread.h"

static rt_device_t serial = RT_NULL;
struct rt_semaphore NiMingFlow_sem; /* 用于接收消息的信号量 */
uint8_t NiMingFlow_rx_buffer[RT_SERIAL_RB_BUFSZ + 1];

static char msg_pool[256];

/* 串口设备句柄 */
static rt_device_t serial;
/* 消息队列控制块 */
static struct rt_messagequeue NiMingFlow_rx_mq;

/*匿名光流接收数据结构*/
NiMingFlow_Rec NiMingFlow_data = {0};

NiMingFlow_Raw Nimingflow_1 = {0};
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
    result = rt_mq_send(&NiMingFlow_rx_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}


double sum_x = 0 , sum_y = 0 ,k1 = 0 , k2 = 0;
void NiMingFlow_DataProcess(uint8_t *pData,uint8_t rec_length)
{
	//获取数据长度检查
	uint8_t length = pData[3];
	if( length != rec_length - 6 )
	{
		//NiMingFlow_data.data_Valid = 0;
		return ;
	}	
	
	//和校验位校验
	int sum = 0;
	uint8_t check = 0;
	uint16_t i;
	
	for(i = 0;i < rec_length - 2;i++)
	{
			sum += pData[i];
	}
	check =(uint8_t)sum & 0xFF;
	if(check != pData[rec_length-2] )
	{
		//NiMingFlow_data.data_Valid = 0;
		return ;
	}
	
	uint8_t ID = pData[2];
	if(ID == 0x51)//光流数据
	{
//		Nimingflow_1.Vx_flow = pData[6];
//		Nimingflow_1.Vy_flow = pData[7];
//		NiMingFlow_data.Vx_Flow = ((uint16_t)pData[17] | ((uint16_t)pData[18] << 8));
//		NiMingFlow_data.Vy_Flow = ((uint16_t)pData[19] | ((uint16_t)pData[20] << 8));
//		
//		if(NiMingFlow_data.distance != 0)
//		{	if(Nimingflow_1.Vx_flow != 0)
//				k1 = NiMingFlow_data.Vx_Flow*0.02 / Nimingflow_1.Vx_flow / NiMingFlow_data.distance*1000;
//			if(Nimingflow_1.Vy_flow != 0)
//				k2 = NiMingFlow_data.Vy_Flow*0.02 / Nimingflow_1.Vy_flow / NiMingFlow_data.distance*1000;
//		}
//		sum_x +=  NiMingFlow_data.Vx_Flow*0.02;
//		sum_y +=  NiMingFlow_data.Vy_Flow*0.02;
				
		uint8_t mode = pData[4];
		uint8_t state = pData[5];
		
		if(!state)//状态错误
		{
			NiMingFlow_data.pos_data_Valid = 0;
			return ;
		}
		
		if(mode == 2)
		{
			NiMingFlow_data.Vx_Flow =  ((uint16_t)pData[9] | ((uint16_t)pData[10] << 8));
			NiMingFlow_data.Vy_Flow =  ((uint16_t)pData[11] | ((uint16_t)pData[12] << 8));
			NiMingFlow_data.pos_x   =  ((uint16_t)pData[13] | ((uint16_t)pData[14] << 8));
			NiMingFlow_data.pos_y   =  ((uint16_t)pData[15] | ((uint16_t)pData[16] << 8));
			NiMingFlow_data.quality =  pData[17];
            
            if(NiMingFlow_data.quality > 150)
            {
                NiMingFlow_data.pos_data_Valid = 1;
                NiMingFlow_data.pos_data_fresh_time = rt_tick_get(); // 刷新数据的更新时间
            }
            else NiMingFlow_data.pos_data_Valid = 0;

		}else if(mode ==0)
		{
			Nimingflow_1.Vx_flow = pData[6];
			Nimingflow_1.Vy_flow = pData[7];
		    Nimingflow_1.quality = pData[8];
			sum_x += Nimingflow_1.Vx_flow;
			sum_y += Nimingflow_1.Vy_flow;
		}	else if(mode == 1)
		{
			Nimingflow_1.Vx_flow = ((uint16_t)pData[6] | ((uint16_t)pData[7] << 8));
			Nimingflow_1.Vy_flow = ((uint16_t)pData[8] | ((uint16_t)pData[9] << 8));
			sum_x += Nimingflow_1.Vx_flow*0.02;
			sum_y += Nimingflow_1.Vy_flow*0.02;
		}
	}else if(ID == 0x34 )
	{
        static int16_t first_flag = 0;
        static float distance_last;
		uint32_t distance = (uint32_t)pData[7] | ((uint32_t)pData[8] << 8) | ((uint32_t)pData[9] << 8*2 ) | ((uint32_t)pData[10] << 8*3 );

		if(distance != 0xFFFFFFFF )
		{
			NiMingFlow_data.distance = distance;
			NiMingFlow_data.height_data_Valid = 1;
            NiMingFlow_data.height_data_fresh_time = rt_tick_get(); // 刷新数据的更新时间
            if(first_flag != 0)
                NiMingFlow_data.distance_v = (distance - distance_last)/0.02f;//使用高度变化量估计速度
            else 
                first_flag++;
            distance_last = distance;
		}
		else NiMingFlow_data.height_data_Valid = 0;
	}

}



static void NM_serial_thread_entry(void *parameter)
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
        result = rt_mq_recv(&NiMingFlow_rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, NiMingFlow_rx_buffer, msg.size);
						
            if (rx_length <= 6)
            { // 如果长度不对，则直接跳过，但是必须从rt_device_read读出，否则缓冲区会溢出
                continue;
            }
            NiMingFlow_rx_buffer[rx_length] = '\0';

            NiMingFlow_DataProcess(NiMingFlow_rx_buffer,rx_length);
						
						
     //释放数据处理信号量
     //   while (rt_sem_trytake(&NiMingFlow_sem) == RT_EOK)
     //       continue;
     //   rt_sem_release(&NiMingFlow_sem);
    }
}
}


rt_err_t NiMingFlow_Init(void)
{
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(NiMingFlow_device);

    /* step2：修改串口配置参数 */
    config.baud_rate = 500000;      //修改波特率
    config.data_bits = DATA_BITS_8; //数据位 8
    config.stop_bits = STOP_BITS_1; //停止位 1
    config.bufsz = 128;             //修改缓冲区 buff size 为 128
    config.parity = PARITY_EVEN;    //

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* 初始化消息队列 */
    rt_mq_init(&NiMingFlow_rx_mq, "rx_mq",
               msg_pool,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX );
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);

    //初始化信号量
    rt_sem_init(&NiMingFlow_sem, "NiMingFlow_rec", 0, RT_IPC_FLAG_FIFO);

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", NM_serial_thread_entry, RT_NULL, 2048, THREAD_PRIO_SENSOR_UART_RX, 5);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
   
   return RT_EOK;
}
