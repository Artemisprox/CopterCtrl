#include "drv_IMUPosCali.h"
#include "func_IMUCali.h"
#include "drv_thread.h"
#include "drv_HWTimer.h"
#include "func_SensorRAW.h"
#include "func_TempCtr.h"
#include "drv_RGB.h"
#include "drv_gpio.h"
#include <rtthread.h>
#include "drv_IMU.h"
#include <rtdevice.h>

static struct rt_semaphore IMU1_PosCaliFinish_Sem; // 通信结束后通知数据处理线程处理数据
//static struct rt_semaphore IMU2_PosCaliFinish_Sem; // 通信结束后通知数据处理线程处理数据
static rt_thread_t IMU_PosCali_Tid = RT_NULL; // IMU校准线程

static double Pos_Sum[3] = {0};         // x、y、z三轴的校准求和数值
IMU_InstallPos IMU1_pos;

/* 引脚编号，通过查看设备驱动文件drv_gpio.c确定 */
#define KEY1_PIN_NUM GET_PIN(B, 9)
#define KEY2_PIN_NUM GET_PIN(C, 10)

void IMU_PosCali_Thread(void *Para)
{   
    static int16_t PosCount = 0;
    while(1)
    {
        IMU_WaitForRawData();
        if(PosCount < IMUDataColletNum )
        {
            Pos_Sum[0] +=  HERO_IMU.pitch;
            Pos_Sum[1] +=  HERO_IMU.roll;
            PosCount++;
        }else
        {
            IMU1_pos.Pitch = Pos_Sum[0]/1.0f/IMUDataColletNum;
            IMU1_pos.Roll = Pos_Sum[1]/1.0f/IMUDataColletNum;
            rt_sem_release(&IMU1_PosCaliFinish_Sem);
        }
    }
    
}

static void RGB_R_TEST(void *args)
{
  static rt_uint8_t first_flag = 0;
	//rt_thread_mdelay(10);
	uint8_t p;
	p = rt_pin_read(KEY1_PIN_NUM);
    if (!p)
    {
////        /*蜂鸣器提示进入IMU校准模式*/
////        rt_pin_write(BEEP_PIN_NUM, PIN_HIGH);
////        rt_thread_mdelay(20);
////        rt_pin_write(BEEP_PIN_NUM, PIN_LOW);
////        green_keepon();
////			
		if(!first_flag)	
		{
			first_flag++;
			
			IMU_PosCali_Tid = rt_thread_create(
        "IMUPos_Thread",                 // 线程名
        IMU_PosCali_Thread,      // 线程入口
        RT_NULL,                  // 入口参数无
        512,                      // 线程栈
        THREAD_PRIO_IMU_POSCALI, // 线程优先级
        1);                       // 线程时间片大小

			// 线程创建失败返回false
			if (IMU_PosCali_Tid == RT_NULL)
			{
					return;
			}
			rt_thread_startup(IMU_PosCali_Tid);
		}
    }
}

void IMU_PosCali_Init(void)
{
//    /* 蜂鸣器引脚为输出模式 */
//    rt_pin_mode(BEEP_PIN_NUM, PIN_MODE_OUTPUT);
//    /* 默认低电平 */
//    rt_pin_write(BEEP_PIN_NUM, PIN_LOW);

//    /* 按键0引脚为输入模式 */
//    rt_pin_mode(KEY1_PIN_NUM, PIN_MODE_INPUT_PULLUP);
//    /* 绑定中断，下降沿模式，回调函数名为beep_on */
//    rt_pin_attach_irq(KEY1_PIN_NUM, PIN_IRQ_MODE_FALLING, RGB_R_TEST, RT_NULL);
//    /* 使能中断 */
//    rt_pin_irq_enable(KEY1_PIN_NUM, PIN_IRQ_ENABLE);

	// 用来挂起的信号量，校准线程运行结束后会释放
	rt_sem_init(&IMU1_PosCaliFinish_Sem, "IMU1TriS", 0, RT_IPC_FLAG_PRIO);

	IMU_PosCali_Tid = rt_thread_create(
        "IMUPos_Thread",                 // 线程名
        IMU_PosCali_Thread,      // 线程入口
        RT_NULL,                  // 入口参数无
        512,                      // 线程栈
        THREAD_PRIO_IMU_POSCALI, // 线程优先级
        1);                       // 线程时间片大小

			// 线程创建失败返回false
			if (IMU_PosCali_Tid == RT_NULL)
			{
					return;
			}
	rt_thread_startup(IMU_PosCali_Tid);
	
    rt_sem_take(&IMU1_PosCaliFinish_Sem,RT_WAITING_FOREVER);
    rt_thread_delete(IMU_PosCali_Tid);
}
