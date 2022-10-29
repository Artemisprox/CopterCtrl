#ifndef __HTHREAD_DATA_H__
#define __HTHREAD_DATA_H__

//监视器线程	
#define THREAD_STACK_MONITOR 		(1024)
#define THREAD_PRIO_MONITOR 		(1)
#define THREAD_TICK_MONITOR 		(2)
//单轮pid线程
#define THREAD_STACK_WHEELS_CAL 	(768)
#define THREAD_PRIO_WHEELS_CAL 	    (1)
#define THREAD_TICK_WHEELS_CAL 	    (2)	
//can1线程
#define THREAD_STACK_CAN1RX 		(768)
#define THREAD_PRIO_CAN1RX 			(2)
#define THREAD_TICK_CAN1RX 			(5)
//can2线程	
#define THREAD_STACK_CAN2RX 		(768)
#define THREAD_PRIO_CAN2RX 			(3)
#define THREAD_TICK_CAN2RX 			(5)
//云台通信线程	
#define THREAD_STACK_GIMBAL 		(768)
#define THREAD_PRIO_GIMBAL 			(4)
#define THREAD_TICK_GIMBAL 			(2)
//底盘控制线程
#define THREAD_STACK_CHASSIS_CTRL   (1024)
#define THREAD_PRIO_CHASSIS_CTRL 	(5)
#define THREAD_TICK_CHASSIS_CTRL 	(2)	
//裁判系统线程	
#define THREAD_STACK_DJI 			(512)
#define THREAD_PRIO_DJI 			(7)
#define THREAD_TICK_DJI 			(10)
//超级电容通信线程	
#define THREAD_STACK_SC 			(768)
#define THREAD_PRIO_SC 				(9)
#define THREAD_TICK_SC 				(2)
//平滑线程	
#define THREAD_STACK_SMOOTH         (768)
#define THREAD_PRIO_SMOOTH		    (10)
#define THREAD_TICK_SMOOTH 	        (5)
//自定义UI线程	
#define THREAD_STACK_UI 			(1024)
#define THREAD_PRIO_UI 				(15)
#define THREAD_TICK_UI 				(1)
//机器人间通信线程	
#define THREAD_STACK_INTERACT   	(1024)
#define THREAD_PRIO_INTERACT    	(16)
#define THREAD_TICK_INTERACT    	(1)
//测试线程	
#define THREAD_STACK_MYTEST 		(512)
#define THREAD_PRIO_MYTEST			(17)
#define THREAD_TICK_MYTEST			(5)
//遥控器线程	
#define THREAD_STACK_REMOTE 		(1024)
#define THREAD_PRIO_REMOTE 			(22)
#define THREAD_TICK_REMOTE 			(2)
//debug 接收线程	
#define THREAD_STACK_DEBUGRX 		(1024)
#define THREAD_PRIO_DEBUGRX     	(23)
#define THREAD_TICK_DEBUGRX     	(2)
//debug 发送线程	
#define THREAD_STACK_DEBUGTX 		(1024)
#define THREAD_PRIO_DEBUGTX     	(24)
#define THREAD_TICK_DEBUGTX     	(2)
//报警线程	
#define THREAD_STACK_ALARM 			(1024)
#define THREAD_PRIO_ALARM 			(26)
#define THREAD_TICK_ALARM 			(2)


#endif
