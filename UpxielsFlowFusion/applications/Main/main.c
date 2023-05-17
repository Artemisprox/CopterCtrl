#include <rtthread.h>
#include "rtdef.h"
#include "func_uart_rec.h"
#include "app_Data_Send.h"
#include "drv_canthread.h"

int main(void)
{
    UART_REC_Init();
		can_init();
}
