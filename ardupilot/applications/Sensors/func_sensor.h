#include <rtthread.h>

#define k 0.98f
#define PERIOD 0.02f
#define DATA_FUSE 0
#define USING_FLOW 1

typedef struct
{
    float distance;
    float V_height;
    float pos_x;
    float V_pos_x;
    float pos_y;
    float V_pos_y; 

    uint8_t pos_valid;
    uint8_t height_valid;
}pos_sensor;

typedef struct 
{
    float acc_x;
    float acc_y;
    float acc_z;
}acc_sensor;

typedef struct 
{
    uint32_t pos_time;
    uint32_t height_time;
}data_fresh_time;

//extern pos_sensor copter_pos;
