#include "rtdevice.h"

#include <rtdevice.h>

#include "drv_icm20602.h"

rt_err_t IMU_TEST_Init(void)
{
    ICM_init();
    icm20602_get_gyro_IMU2();
}
