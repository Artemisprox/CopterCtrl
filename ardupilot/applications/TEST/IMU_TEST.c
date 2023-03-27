#include "rtdevice.h"

#include <rtdevice.h>

#include "drv_icm20602.h"

float gyro[3];
rt_err_t IMU_TEST_Init(void)
{
    ICM_init();
    while (1)
    {
        icm20602_get_gyro_IMU2(gyro);
        rt_thread_mdelay(10);
    }
}
