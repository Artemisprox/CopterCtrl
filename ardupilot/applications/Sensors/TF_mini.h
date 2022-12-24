#include <rtthread.h>

#define TF_mini_device "uart1"


typedef struct
{
	uint16_t distance;
	uint16_t strength;
	uint16_t temperature;
	uint8_t data_num;
	uint8_t Data_valid;
	uint8_t Data_fresh_time;
} TF_mini_rec;

extern TF_mini_rec TF_mini_data;
rt_err_t TF_mini_Init(void);
