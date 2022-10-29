#ifndef __DRV_ADC_H__
#define __DRV_ADC_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/* ADC1外设含自动初始化 */

//用于ADC输出电压换算
#ifndef HW_VOLTAGE_SUPPLY
#define HW_VOLTAGE_SUPPLY (3.3F)
#endif

#define ADC_SAMPLE_PERIOD (100)//设置每一轮ADC采样的周期，单位us

#define ADC_SAMPLE_TIMES (1)// 设置每次DMA中断时处理多少次采样的数据

#define ADC_BUFF_LEN (ADC_SAMPLE_TIMES)//缓冲区长度需要和adc循环采样时的循环次数相等

#define ADC_FILTER_RATE (0.10f) //滤波过程中对历史数据的计算比例

extern char ADC_Fresh; // 用于看门狗读取的标志位，每次ADC刷新数据时置1，看门狗读取时清零

extern float Sply_Voltage; //adc会结合Vref测得当前单片机的供电电压

//接口：读取adc电压值，已经进行过软件RC滤波
extern float adc_read(char channel);

//接口：初始化ADC定时采样滤波, 初始化完成后会delay等待一段时间，确保adc滤波已经稳定工作
extern void adc_dev_init(void);

#endif
