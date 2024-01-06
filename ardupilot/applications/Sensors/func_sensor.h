#include <rtthread.h>

#define PERIOD 0.02f
#define DATA_FUSE 1
#define SIMPLE_FUSE	2 
#define NO_FUSE 3
#define USING_FLOW 1
#define test 1

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

extern float get_height(void);
extern float get_v_height(void);
extern rt_err_t Sensor_Init(void);
//extern pos_sensor copter_pos;
