#include <rtthread.h>

#define STABILIZATION    1   //自稳模式
#define HEIGHT           2   //高度模式
#define POSITION         3   //位置模式

#define READY                 0x01   //空
#define ARMED                 0x00   //起飞
#define FLYING                0x11   //升空
#define LAND                  0x12   //降落
typedef struct 
{
    uint8_t flight_status;
    uint8_t emergency;
    uint8_t recoil_compensate_enable;
    uint8_t mode;

    uint8_t rc_status;

}status;

typedef struct 
{
    uint8_t pos_valid;
    uint8_t height_valid;
    uint8_t atti_valid;
    uint8_t rc_valid;
    uint8_t battery_OK;

    uint8_t copter_OK;
}data_check;
