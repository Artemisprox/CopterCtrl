#ifndef __DRV_THREAD_H__
#define __DRV_THREAD_H__

//线程优先级设定
//<10, 优先于主函数
#define ADC_SAMPLE_THREAD_PRIO 9//adc外设采样及滤波计算 当前adc外设采样过程完全在DMA中断中完成

//>10, 主函数结束后运行
#define CAP_CTRL_APP_THREAD_PRIO 11     //超级电容总控线程
#define CHARGE_P_CTRL_THREAD_PRIO 12    //功率控制计算线程
#define CAN_REC_THREAD_PRIO 15          //CAN通信接收线程
#define CAN_MOD_THREAD_PRIO 16          //CAN通信发送接收处理线程
#define LED_SHO_APP_THREAD_PRIO 18      //LED灯效主控程序
#define OLED_SHO_APP_THREAD_PRIO 20     //显示屏主控程序
#endif
