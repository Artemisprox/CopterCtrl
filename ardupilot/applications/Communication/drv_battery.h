#include <rtthread.h>

#define ADC_DEV_NAME "adc1"
#define REFER_VOLTAGE 5
#define CONVERT_BITS (1 << 12)

#define ADC_BATTERY1 1
#define ADC_BATTERY2 2
#define ADC_BATTERY3 3
#define ADC_BATTERY4 4
#define ADC_BATTERY5 5
#define ADC_BATTERY6 6

#define POWER_ID 0x102

#define BATTERY_LOW_V  11.3f


typedef struct
{
	float voltage;
  	float current;
	uint8_t Battery_data_rec;//是否有电池数据接收
	uint8_t Battery_status;//电池状态是否正常
	uint32_t fresh_time;
}battery;

extern void battery_readmsg(rt_uint8_t rxmsg[]);
extern rt_err_t Battery_Init(void);


