#include <rtthread.h>

#define POWER_ID 0x102

#define BATTERY_LOW  11.3f

typedef struct
{
	float voltage;
  float current;
	uint8_t Battery_data_rec;//是否有电池数据接收
	uint8_t Battery_status;//电池状态是否正常
	uint32_t fresh_time;
}battery;

void battery_readmsg(rt_uint8_t rxmsg[]);
