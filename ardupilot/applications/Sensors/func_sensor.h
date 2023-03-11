#include <rtthread.h>

#define k 0.98
#define period 0.02

typedef struct
{
    float height;
    float V_height;
    float pos_x;
    float V_pos_x;
    float pos_y;
    float V_pos_y; 
}pos_sensor;

typedef struct 
{
    float acc_x;
    float acc_y;
    float acc_z;
}acc_sensor;
