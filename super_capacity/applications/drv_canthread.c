#include "drv_canthread.h"

#define CAN1_DEV_NAME		"can1"				//can设备名称

static struct rt_semaphore can1_rx_sem;     	//用于接收消息的信号量
rt_device_t can1_dev;            		//CAN 设备句柄


//用户需要在func_can.c里重新定义这个函数
//can1数据接收函数
extern void can1_rec(struct rt_can_msg *msg);

/**
 * @brief  can1接收回调
 * @retval RT_EOK
 */
static rt_err_t can1_rx_call(rt_device_t dev, rt_size_t size)
{
    //CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量
    rt_sem_release(&can1_rx_sem);
    return RT_EOK;
}
/**
 * @brief  can1读取线程
 */
static void can1_rx_thread(void *parameter)
{
    struct rt_can_msg rxmsg = {0};

#ifdef RT_CAN_USING_HDR
	rt_err_t res;
	struct rt_can_filter_item items[1] =
		{
			RT_CAN_FILTER_ITEM_INIT(0x095, 0, 0, 1, 0x7FF, RT_NULL, RT_NULL), /* std,match ID:0x200~0x20F，hdr 为 - 1，设置默认过滤表 */
			//RT_CAN_FILTER_ITEM_INIT(0x200, 0, 0, 1, 0x7F0, RT_NULL, RT_NULL), /* std,match ID:0x200~0x20F，hdr 为 - 1，设置默认过滤表 */
			//RT_CAN_FILTER_ITEM_INIT(0x300, 0, 0, 1, 0x700, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
			//RT_CAN_FILTER_ITEM_INIT(0x211, 0, 0, 1, 0x7ff, RT_NULL, RT_NULL), /* std,match ID:0x211，hdr 为 - 1 */
			//RT_CAN_FILTER_STD_INIT(0x486, RT_NULL, RT_NULL),                  /* std,match ID:0x486，hdr 为 - 1 */
			//{0x200, 0, 0, 1, 0x7F0, 14,}                                       /* std,match ID:0x555，hdr 为 14，指定设置 14 号过滤表 */
		};
	struct rt_can_filter_config cfg = {1, 1, items}; /* 一共有 1 个过滤表 */
	/* 设置硬件过滤表 */
	res = rt_device_control(can1_dev, RT_CAN_CMD_SET_FILTER, &cfg);
	RT_ASSERT(res == RT_EOK);
#endif

	while (1)
    {
        //hdr 值为 - 1，表示直接从 uselist 链表读取数据
        rxmsg.hdr = -1;
        //阻塞等待接收信号量
        rt_sem_take(&can1_rx_sem, RT_WAITING_FOREVER);
        //从 CAN 读取一帧数据
        rt_device_read(can1_dev, 0, &rxmsg, sizeof(rxmsg));
        can1_rec(&rxmsg);
    }
}

//CAN1发送函数
int CAN1_Send(rt_uint8_t *DataSource, char DataLength, rt_uint32_t CAN1_Send_ID)
{
	struct rt_can_msg msg = {0};
	rt_size_t size;
//	char can_name[] = "can1";
	int fori;

	if (DataLength > 8)
	{
		while (1); //发送长度不能超过8
	}

	// /* 查找 CAN 设备 */
	// can1_dev = rt_device_find(can_name);
	// if (!can1_dev)
	// {
	// 	rt_kprintf("find %s failed!\n", can_name);
	// 	return RT_ERROR;
	// }

	msg.id = CAN1_Send_ID;	/* ID 原例程为 0x78 */
	msg.ide = RT_CAN_STDID; /* 标准格式 */
	msg.rtr = RT_CAN_DTR;	/* 数据帧 */
	msg.len = DataLength;	/* 数据长度不大于 8 */

	for (fori = 0; fori < DataLength; fori++)
	{
		msg.data[fori] = DataSource[fori];
	}

	/* 发送一帧 CAN 数据 */
	size = rt_device_write(can1_dev, 0, &msg, sizeof(msg));
	if (size == 0)
	{
		rt_kprintf("can dev write data failed!\n");
		return RT_ERROR;
	}
	return RT_EOK;
}

/**
 * @brief  can1初始化，can1数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
int can1_init(void)
{
	rt_err_t res=0;
	rt_thread_t thread;
	can1_dev = rt_device_find(CAN1_DEV_NAME);
	if (!can1_dev)
	{
		return RT_ERROR;
	}
	//can接收中断信号量
	rt_sem_init(&can1_rx_sem, "can1_sem", 0, RT_IPC_FLAG_FIFO);

	//配置can驱动
	res = rt_device_open(can1_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
	RT_ASSERT(res == RT_EOK);
	res = rt_device_control(can1_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
	res = rt_device_control(can1_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
	//设置接收回调函数
	rt_device_set_rx_indicate(can1_dev, can1_rx_call);

	thread = rt_thread_create("can1_rx", can1_rx_thread, RT_NULL, 1024, CAN_REC_THREAD_PRIO, 2);
	if (thread != RT_NULL)
	{
		rt_thread_startup(thread);
	}
	return res; 
}

