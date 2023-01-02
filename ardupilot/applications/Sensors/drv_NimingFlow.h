#include <rtthread.h>

#define NiMingFlow_device "uart6"


typedef struct
{
	uint32_t distance;
	int16_t Vx_Flow;
	int16_t Vy_Flow;
	int16_t pos_x;
	int16_t pos_y;
	uint8_t  data_Valid;
	uint8_t  Data_fresh_time;
	uint8_t  quality;
} NiMingFlow_Rec;

rt_err_t NiMingFlow_Init(void);

extern NiMingFlow_Rec NiMingFlow_data;

