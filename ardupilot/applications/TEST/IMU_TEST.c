#include "rtdevice.h"
#include "drv_spithread.h"
#include "IMU_TEST.h"

#include <rtdevice.h>

#include "drv_icm20602.h"

static struct rt_spi_message msgACCLRead1, msgACCLRead2, msgACCLRead3;


//读取ID: reg = 0x75
static uint8_t ReadSingle(uint8_t reg)
{
    static uint8_t Accl_Rbuffer[2];
    Accl_Rbuffer[0] = (uint8_t)(reg | 0x80);
    Accl_Rbuffer[1] = 0;

    msgACCLRead1.send_buf = Accl_Rbuffer;
    msgACCLRead3.recv_buf = Accl_Rbuffer + 1;

    rt_spi_take_bus(spi_dev_IMU2);
    rt_spi_transfer_message(spi_dev_IMU2, &msgACCLRead1);
    rt_spi_release_bus(spi_dev_IMU2);
    return Accl_Rbuffer[1];
}

static uint8_t WriteSingle(uint8_t reg)
{
    static uint8_t Accl_Rbuffer[2];
    Accl_Rbuffer[0] = (uint8_t)(reg | 0x7f);
    Accl_Rbuffer[1] = 0;

    msgACCLRead1.send_buf = Accl_Rbuffer;
    msgACCLRead3.recv_buf = Accl_Rbuffer + 1;

    rt_spi_take_bus(spi_dev_IMU2);
    rt_spi_transfer_message(spi_dev_IMU2, &msgACCLRead1);
    rt_spi_release_bus(spi_dev_IMU2);
    return Accl_Rbuffer[1];
}

float gyro[3];
rt_err_t IMU_Init(void)
{
   // ICM_init();
    spi_ICM20602_init();
    WriteSingle(0x80);
    WriteSingle(0x01);
    while (1)
    {
        //icm20602_get_gyro_IMU2(gyro);
        ReadSingle(0x75);
        rt_thread_mdelay(10);
    }
}
