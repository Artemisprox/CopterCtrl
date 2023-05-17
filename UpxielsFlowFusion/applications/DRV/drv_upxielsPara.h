#include <rtthread.h>

typedef struct 
{
    int16_t flow_x_integral;
    int16_t flow_y_integral;
    int16_t integration_timespan;
		uint8_t quality;
    float x_radians;
    float y_radians;
}upxiels_rawdata;

typedef enum
{
    flow_front = 1,
    flow_behind,
    flow_right,
    flow_left
}flow_num;

extern void UP_Flow_Process(uint8_t *pData,uint8_t rec_length , upxiels_rawdata* flow_data );
