#include "CAN_TEST.h"

#include <rtthread.h>
#include <board.h>

//#include "UWB.h"

#define TEST_CAN1_DEV_NAME "can1" // can设备名称
#define TEST_CAN2_DEV_NAME "can2"

static struct rt_semaphore TEST_can1_rx_sem; //用于接收消息的信号量
rt_device_t TEST_can1_dev;                   // CAN 设备句柄

static struct rt_semaphore TEST_can2_rx_sem; //用于接收消息的信号量
rt_device_t TEST_can2_dev;                   // CAN 设备句柄

struct rt_can_msg rxmsg_can1[0x100] = {0};
struct rt_can_msg rxmsg_can2[0x100] = {0};

/**
 * @brief  TEST_can1接收回调
 * @retval RT_EOK
 */
static rt_err_t TEST_can1_rx_call(rt_device_t dev, rt_size_t size)
{
    // CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量
   // rt_sem_release(&TEST_can1_rx_sem);
    return RT_EOK;
}
/**
 * @brief  TEST_can1读取线程
 */
static void TEST_can1_rx_thread(void *parameter)
{
    rt_int32_t count = 0;
    while (1)
    {
        // hdr 值为 - 1，表示直接从 uselist 链表读取数据
        rxmsg_can1[count].hdr = -1;
        //阻塞等待接收信号量
        rt_sem_take(&TEST_can1_rx_sem, RT_WAITING_FOREVER);
        //从 CAN 读取一帧数据
        rt_device_read(TEST_can1_dev, 0, &rxmsg_can1[count], sizeof(rxmsg_can1[count]));
        ++count;
    }
}
/**
 * @brief  TEST_can2接收回调
 * @retval RT_EOK
 */
static rt_err_t TEST_can2_rx_call(rt_device_t dev, rt_size_t size)
{
    // CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量
    //rt_sem_release(&TEST_can2_rx_sem);
    return RT_EOK;
}
/**
 * @brief  TEST_can2读取线程
 */
static void TEST_can2_rx_thread(void *parameter)
{
    rt_int32_t count = 0;
    while (1)
    {
        // hdr 值为 - 1，表示直接从 uselist 链表读取数据
        rxmsg_can2[count].hdr = -1;
        //阻塞等待接收信号量
        rt_sem_take(&TEST_can2_rx_sem, RT_WAITING_FOREVER);
        //从 CAN 读取一帧数据
        rt_device_read(TEST_can2_dev, 0, &rxmsg_can2[count], sizeof(rxmsg_can2[count]));
        ++count;
    }
}

/**
 * @brief  TEST_can1初始化，TEST_can1数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
static int TEST_can1_init(void)
{
    rt_err_t res = 0;
    rt_thread_t thread;
    // can接收中断信号量
    rt_sem_init(&TEST_can1_rx_sem, "TEST_can1_sem", 0, RT_IPC_FLAG_FIFO);

#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[2] =
        {
            RT_CAN_FILTER_ITEM_INIT(0x100, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff，hdr 为 - 1，设置默认过滤表 */
            RT_CAN_FILTER_ITEM_INIT(0x200, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
        };
    struct rt_can_filter_config cfg = {2, 1, items}; /* 一共有 5 个过滤表 */
    /* 设置硬件过滤表 */
    res = rt_device_control(TEST_can1_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif

    thread = rt_thread_create("TEST_can1_rx", TEST_can1_rx_thread, RT_NULL, 2048, THREAD_PRIO_TEST_CAN1RX, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    TEST_can1_dev = rt_device_find(TEST_CAN1_DEV_NAME);
    if (!TEST_can1_dev)
        return RT_ERROR;
    //配置can驱动
    res = rt_device_open(TEST_can1_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    RT_ASSERT(res == RT_EOK);
    res = rt_device_control(TEST_can1_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    res = rt_device_control(TEST_can1_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    //设置接收回调函数
    rt_device_set_rx_indicate(TEST_can1_dev, TEST_can1_rx_call);
    return res;
}
/**
 * @brief  TEST_can2初始化，TEST_can2数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
static int TEST_can2_init(void)
{
    rt_err_t res = 0;
    rt_thread_t thread;
    // can接收中断信号量
    rt_sem_init(&TEST_can2_rx_sem, "TEST_can2_sem", 0, RT_IPC_FLAG_FIFO);

#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[3] =
        {
            RT_CAN_FILTER_ITEM_INIT(0x200, 0, 0, 1, 0x7F0, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff，hdr 为 - 1，设置默认过滤表 */
            RT_CAN_FILTER_ITEM_INIT(0x020, 0, 0, 1, 0x7F0, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
            RT_CAN_FILTER_ITEM_INIT(0x010, 0, 0, 1, 0x7FF, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
        };
    struct rt_can_filter_config cfg = {3, 1, items}; /* 一共有 5 个过滤表 */
    /* 设置硬件过滤表 */
    res = rt_device_control(TEST_can2_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif

    thread = rt_thread_create("TEST_can2_rx", TEST_can2_rx_thread, RT_NULL, 2048, THREAD_PRIO_TEST_CAN2RX, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    TEST_can2_dev = rt_device_find(TEST_CAN2_DEV_NAME);
    if (!TEST_can2_dev)
        return RT_ERROR;
    //配置can驱动
    res = rt_device_open(TEST_can2_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    RT_ASSERT(res == RT_EOK);
    res = rt_device_control(TEST_can2_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    res = rt_device_control(TEST_can2_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    //设置接收回调函数
    rt_device_set_rx_indicate(TEST_can2_dev, TEST_can2_rx_call);
		
    return res;
}

static void TEST_CAN_Ctrl(void *parameter)
{
    struct rt_can_msg can_msg = {0};
    rt_int32_t count = 0;

    can_msg.id = 0x001;              /* ID 为 0x78 */
    can_msg.ide = RT_CAN_STDID;     /* 标准格式 */
    can_msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    can_msg.len = 8;                /* 数据长度为 8 */
    /* 待发送的 8 字节数据 */
    can_msg.data[0] = 0x01;
    can_msg.data[1] = 0x01;
    can_msg.data[2] = 0x05;
    can_msg.data[3] = 0x01;
    can_msg.data[4] = 0x01;
    can_msg.data[5] = 0x01;
    can_msg.data[6] = 0x01;
    can_msg.data[7] = 0x01;

    while(1)
    {
        if(count < 0xFF)
        {
            rt_device_write(TEST_can1_dev, 0, &can_msg, sizeof(can_msg));
            rt_device_write(TEST_can2_dev, 0, &can_msg, sizeof(can_msg));
            //can_msg.data[0] ++;
            //++count;
        }
        rt_thread_mdelay(1);
    }
}

static rt_err_t TEST_CANCtrl_init(void)
{
    rt_thread_t thread;

    thread = rt_thread_create("TEST_CAN_Ctrl", TEST_CAN_Ctrl, RT_NULL, 2048, THREAD_PRIO_TEST_CAN2RX, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
    return RT_EOK;
}

rt_err_t CAN_Init(void)
{
    TEST_can1_init();
    TEST_can2_init();
    TEST_CANCtrl_init();
    return RT_EOK;
}
