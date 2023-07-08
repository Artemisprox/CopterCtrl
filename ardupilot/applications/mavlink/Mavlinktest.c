#include <mavlink.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_thread.h"
#include "Mavlinktest.h"
#include "drv_IMU.h"

static rt_device_t serial = RT_NULL;
mavlink_message_t Mavlink_rx_buffer;
uint8_t Mavlink_tx_buffer[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_IMU_DATA_LEN + 1];
mavlink_imu_data_t test_data;

static char msg_pool[512];

/* 消息队列控制块 */
static struct rt_messagequeue Mavlink_rx_mq;

/* 串口接收消息结构*/
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

rt_err_t mavlink_imudata_send(IMU_t data)
{
    mavlink_message_t message_buff;
    mavlink_msg_imu_data_pack(18,18,&message_buff,data.atti_ready,data.speed_ready,data.pitch,data.roll,data.yaw);
    int length = 0;
    length = mavlink_msg_to_send_buffer(Mavlink_tx_buffer,&message_buff);

    if(length)
        return RT_EOK;
    else return RT_ERROR;
}

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;
    result = rt_mq_send(&Mavlink_rx_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}

mavlink_status_t status;
mavlink_channel_t chan;
static void Mavlink_serial_thread_entry(void *parameter)
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
        result = rt_mq_recv(&Mavlink_rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, &Mavlink_rx_buffer, msg.size);
						chan = MAVLINK_COMM_0;
            if(mavlink_parse_char(chan , rx_length , &Mavlink_rx_buffer , &status))
            {
                mavlink_msg_imu_data_decode( &Mavlink_rx_buffer , &test_data );
            }
    }
}
}
    

//发送函数供给mavlink进行调用
void rtt_mavlink_write(const char *buf, uint16_t len)
{
    rt_device_write(serial, 0, buf, (unsigned long)(len));
}

rt_err_t Mavlink_Init(void)
{
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */

		/*接收串口*/
    /* step1：查找串口设备 */
    serial = rt_device_find(Mavlink_rec_com);

    /* step2：修改串口配置参数 */
    config.baud_rate = BAUD_RATE_115200;  //修改波特率
    config.data_bits = DATA_BITS_8; //数据位 8
    config.stop_bits = STOP_BITS_1; //停止位 1
    config.bufsz = 128;             //修改缓冲区 buff size 为 128
    config.parity = PARITY_EVEN;    //

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* 初始化消息队列 */
    rt_mq_init(&Mavlink_rx_mq, "rx_mq",
               msg_pool,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX );
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);
							 
							 
		/*发送串口*/					 
		/* step1：查找串口设备 */
    serial = rt_device_find(Mavlink_send_com);

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX );

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", Mavlink_serial_thread_entry, RT_NULL, 2048, THREAD_PRIO_SENSOR_UART_RX, 5);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
   
   return RT_EOK;
}
