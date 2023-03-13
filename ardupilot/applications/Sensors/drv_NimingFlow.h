#include <rtthread.h>

#define NiMingFlow_device "uart6"


typedef struct
{
	uint32_t distance;
	float distance_v;
	int16_t Vx_Flow;
	int16_t Vy_Flow;
	int16_t pos_x;
	int16_t pos_y;
	
	uint8_t  pos_data_Valid;
	uint8_t  height_data_Valid;
	uint32_t height_data_fresh_time;
	uint32_t pos_data_fresh_time;
	uint8_t  quality;
} NiMingFlow_Rec;

typedef struct
{
	int8_t Vx_flow;
	int8_t Vy_flow;
	uint8_t quality;
}NiMingFlow_Raw;

rt_err_t NiMingFlow_Init(void);

extern NiMingFlow_Rec NiMingFlow_data;
