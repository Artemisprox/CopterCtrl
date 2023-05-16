#include <rtthread.h>
#include "rtdef.h"

#include "drv_CustCtrler_Data.h"
#include "func_uart_rec.h"
#include "app_Data_Send.h"

int main(void)
{
    CustCtrler_Datanum_Find();
    if (UART_REC_Init() != RT_EOK)
    {
        while (1)
            continue;
    }
    if (UART_Send_Init() != RT_EOK)
    {
        while (1)
            continue;
    }

    return RT_EOK;
}
