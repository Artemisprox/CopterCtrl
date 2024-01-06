#include "INS_FLOW.h"
#include <rtdevice.h>
#include "velocity_estimator.h"
#include "func_sensor.h"
#include "roboselect.h"
#include "drv_dataserve.h"
#include "drv_IMU.h"
#include "func_SensorRAW.h"

pos_sensor copter_pos;
extern uint8_t test_flag;
JustFloat ins_flow_data;
extern Sensor_RAW_t copter_IMU_RAW;
struct rt_semaphore sem;
const uint8_t just_float_tail[4] = {0x00, 0x00, 0x80, 0x7f};

#define SAMPLE_UART_NAME       "uart5"

rt_device_t serial;

void test_tim_1ms_IRQHandler(void *paramete)
{
	rt_sem_release(&sem);
}

void test_thread_entry(void *parameter)
{

    while (1)
    {
		  rt_sem_take(&sem, RT_WAITING_FOREVER);
			rt_device_write(serial, 0, ins_flow_data.cha, NUOFDATA*4);
			rt_device_write(serial, 0, just_float_tail, 4);
    }
}

static struct rt_timer tim;
int Test_UART_Init(void)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];

    rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);
 
    serial = rt_device_find(uart_name);

    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);

	  rt_sem_init(&sem, "sem", 0, RT_IPC_FLAG_FIFO);
	  rt_thread_t test_thread = rt_thread_create("serial", test_thread_entry, RT_NULL, 1024, 23, 1);

    rt_timer_init(&tim, "integral_tim", test_tim_1ms_IRQHandler, RT_NULL, 1,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

//    rt_timer_start(&tim);
//		rt_thread_startup(test_thread);

    return ret;
}
