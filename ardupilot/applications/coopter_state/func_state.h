#include <rtthread.h>

#define stabilization    1   
#define height           2   
#define position         3   

#define ready                 1   
#define armed                 0   
#define emergency_stop        2   

typedef struct 
{
    uint8_t flight_status;
    uint8_t emergency;
    
    uint8_t mode;

}status;

typedef struct 
{
    uint8_t pos_valid;
    uint8_t height_valid;
    uint8_t atti_valid;
    uint8_t battery_OK;
}data_check;
