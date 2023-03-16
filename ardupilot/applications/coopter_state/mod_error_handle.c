#include "mod_error_handle.h"
#include "drv_RGB.h"

LoopQueueCTRL_Type error_list;

void error_write(int error_ID)
{
    int *p;
    p = (int*)Queue_GetWriteNum(&error_list);
    *p = error_ID;
}

static void error_handle(int error_ID)
{
    switch (error_ID)
    {
    case ATTI_LOST:
        red_keepon();
        rt_kprintf("IMU communication lost");
        break;
    case RC_LOST:
        red_blink_slowly();
        rt_kprintf("Remote connection lost");
        break;
    case BATTERY_LOST:
        red_keepon();
        rt_kprintf("Battery connect lost");
        break;
    case HEIGHT_LOST:
        yellow_quickly();
        rt_kprintf("No height data");
        break;
    case POS_LOST:
        yellow_slowly();
        rt_kprintf("No height data");
        break;
    case BATTERY_LOW:
        yellow_keepon();
        rt_kprintf("Battery capacity low");
        break;
    case THROTTLE_HIGH:
        red_threetimes();
        rt_kprintf("Rockers aren't in the primary position");
        break;
    default:
        green_keepon();
        break;
    }

}

void error_read(void)
{   
    int* read_p;
    while (error_list.Valid_Data)
    {
        read_p = (int*)Queue_Get_ReadEnd(&error_list);
        error_handle(*read_p);
        Queue_Delete_End(&error_list);
    }
}


void error_handle_init(void)
{
    QueueCtrl_Init(&error_list , LIST_LEN);
    RGB_init();
}
