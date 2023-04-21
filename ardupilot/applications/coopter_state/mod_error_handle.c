#include "mod_error_handle.h"
#include "drv_RGB.h"

LoopQueueCTRL_Type error_list;
int error_ID_list[LIST_LEN];
//错误写入
void error_write(int error_ID)
{
    int p;
    p = Queue_GetWriteNum(&error_list);
    error_ID_list[p] = error_ID;
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
    case CHECK_BATTERY:
        red_keepon();
        rt_kprintf("Battery connect lost");
        break;
    case HEIGHT_LOST:
        blue_quickly();
        rt_kprintf("No height data");
        break;
    case POS_LOST:
        blue_slowly();
        rt_kprintf("No height data");
        break;
    case BATTERY_LOW:
        blue_keepon();
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


//错误集中报错:选取最严重的一个错误进行报错
void error_read(void)
{   
    int read , terrible_error;
    terrible_error = Queue_Get_ReadEnd(&error_list);
    while (error_list.Valid_Data)
    {
        read = Queue_Get_ReadEnd(&error_list);
        if( error_ID_list[terrible_error] >= error_ID_list[read])
            terrible_error = read;
        Queue_Delete_End(&error_list);
    }
    error_handle(error_ID_list[terrible_error]);
}


void error_handle_init(void)
{
    QueueCtrl_Init(&error_list , LIST_LEN);
   // MainRGB_init();
}
