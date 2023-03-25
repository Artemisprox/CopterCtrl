#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "drv_spithread.h"
#include "drv_icm20602.h"
#include "math.h"

//========ICM20602寄存器地址========================
/********************************************
*复位后所有寄存器地址都为0，除了
*Register 26  CONFIG				= 0x80
*Register 107 Power Management 1 	= 0x41
*Register 117 WHO_AM_I 				= 0x12
*********************************************/
//陀螺仪温度补偿
#define	ICM20_XG_OFFS_TC_H				0x04
#define	ICM20_XG_OFFS_TC_L				0x05
#define	ICM20_YG_OFFS_TC_H				0x07
#define	ICM20_YG_OFFS_TC_L				0x08
#define	ICM20_ZG_OFFS_TC_H				0x0A
#define	ICM20_ZG_OFFS_TC_L				0x0B
//加速度自检输出(出产时设置，用于与用户的自检输出值比较）
#define	ICM20_SELF_TEST_X_ACCEL			0x0D
#define	ICM20_SELF_TEST_Y_ACCEL			0x0E
#define	ICM20_SELF_TEST_Z_ACCEL			0x0F
//陀螺仪静态偏移
#define	ICM20_XG_OFFS_USRH				0x13
#define	ICM20_XG_OFFS_USRL				0x14
#define	ICM20_YG_OFFS_USRH				0x15
#define	ICM20_YG_OFFS_USRL				0x16
#define	ICM20_ZG_OFFS_USRH				0x17
#define	ICM20_ZG_OFFS_USRL				0x18

#define	ICM20_SMPLRT_DIV				0x19
#define	ICM20_CONFIG					0x1A
#define	ICM20_GYRO_CONFIG				0x1B
#define	ICM20_ACCEL_CONFIG				0x1C
#define	ICM20_ACCEL_CONFIG2				0x1D
#define	ICM20_LP_MODE_CFG				0x1E

//运动唤醒加速度阈值
#define	ICM20_ACCEL_WOM_X_THR			0x20
#define	ICM20_ACCEL_WOM_Y_THR			0x21
#define	ICM20_ACCEL_WOM_Z_THR			0x22


#define	ICM20_FIFO_EN					0x23
#define	ICM20_FSYNC_INT					0x36
#define	ICM20_INT_PIN_CFG				0x37
#define	ICM20_INT_ENABLE				0x38
#define	ICM20_FIFO_WM_INT_STATUS		0x39
#define	ICM20_INT_STATUS				0x3A

//加速度输出
#define	ICM20_ACCEL_XOUT_H				0x3B
#define	ICM20_ACCEL_XOUT_L				0x3C
#define	ICM20_ACCEL_YOUT_H				0x3D
#define	ICM20_ACCEL_YOUT_L				0x3E
#define	ICM20_ACCEL_ZOUT_H				0x3F
#define	ICM20_ACCEL_ZOUT_L				0x40
//温度输出
#define	ICM20_TEMP_OUT_H				0x41
#define	ICM20_TEMP_OUT_L				0x42
//角速度输出
#define	ICM20_GYRO_XOUT_H				0x43
#define	ICM20_GYRO_XOUT_L				0x44
#define	ICM20_GYRO_YOUT_H				0x45
#define	ICM20_GYRO_YOUT_L				0x46
#define	ICM20_GYRO_ZOUT_H				0x47
#define	ICM20_GYRO_ZOUT_L				0x48
//陀螺仪自检输出
#define	ICM20_SELF_TEST_X_GYRO			0x50
#define	ICM20_SELF_TEST_Y_GYRO			0x51
#define	ICM20_SELF_TEST_Z_GYRO			0x52

#define	ICM20_FIFO_WM_TH1				0x60
#define	ICM20_FIFO_WM_TH2				0x61
#define	ICM20_SIGNAL_PATH_RESET			0x68
#define	ICM20_ACCEL_INTEL_CTRL 			0x69
#define	ICM20_USER_CTRL					0x6A
//电源控制
#define	ICM20_PWR_MGMT_1				0x6B
#define	ICM20_PWR_MGMT_2				0x6C

#define	ICM20_I2C_IF					0x70
#define	ICM20_FIFO_COUNTH				0x72
#define	ICM20_FIFO_COUNTL				0x73
#define	ICM20_FIFO_R_W					0x74

#define	ICM20_WHO_AM_I 					0x75
//加速度静态偏移
#define	ICM20_XA_OFFSET_H				0x77
#define	ICM20_XA_OFFSET_L				0x78
#define	ICM20_YA_OFFSET_H				0x7A
#define	ICM20_YA_OFFSET_L				0x7B
#define	ICM20_ZA_OFFSET_H				0x7D
#define	ICM20_ZA_OFFSET_L 				0x7E

//加速度量程
#define ICM20_ACCEL_FS_2G (0 << 3)
#define ICM20_ACCEL_FS_4G (1 << 3)
#define ICM20_ACCEL_FS_8G (2 << 3)
#define ICM20_ACCEL_FS_16G (3 << 3)
//角速度量程
#define ICM20_GYRO_FS_250DPS (0 << 3)
#define ICM20_GYRO_FS_500DPS (1 << 3)
#define ICM20_GYRO_FS_1000DPS (2 << 3)
#define ICM20_GYRO_FS_2000DPS (3 << 3)
// CONFIG DPF
#define DLPF_BW_250 0x00 // Rate=8k
#define DLPF_BW_176 0x01
#define DLPF_BW_92 0x02
#define DLPF_BW_41 0x03
#define DLPF_BW_20 0x04
#define DLPF_BW_10 0x05
#define DLPF_BW_5 0x06
#define DLPF_BW_328 0x06 // Rate=8k
// ACCEL_CONFIG2
#define ACCEL_AVER_4 (0x00 << 4) // Rate=8k
#define ACCEL_AVER_8 (0x01 << 4)
#define ACCEL_AVER_16 (0x02 << 4)
#define ACCEL_AVER_32 (0x03 << 4)
// ACCEL_DLPF
#define ACCEL_DLPF_BW_218 0x00
//#define ACCEL_DLPF_BW_218         	0x01
#define ACCEL_DLPF_BW_99 0x02
#define ACCEL_DLPF_BW_44 0x03
#define ACCEL_DLPF_BW_21 0x04
#define ACCEL_DLPF_BW_10 0x05
#define ACCEL_DLPF_BW_5 0x06
#define ACCEL_DLPF_BW_420 0x06
//===========================================================

#define ICM20602_ADDRESS	0xD2

#define GRAVITY_MSS 	9.80665f							//g转m/s2
#define DEG_TO_RAD    0.0174532f 							//度转弧度
#define RAD_TO_DEG    57.29578f								//度转弧度

#define SPI_DEVICE_NAME     "spi20"

//struct  rt_spi_device *spi_dev;		/* spi 设备句柄 */

static uint8_t icm20602_write_reg(uint8_t reg, uint8_t val , struct  rt_spi_device *spi_dev)
{

    static uint8_t ICM_Tx1, ICM_Tx2;

    ICM_Tx1 = reg&0x7f;
    ICM_Tx2 = val;

    rt_spi_send_then_send(spi_dev, &ICM_Tx1, 1, &ICM_Tx2, 1);

    return 0;
}

static uint8_t icm20602_read_reg(uint8_t reg , struct  rt_spi_device *spi_dev)
{
    static uint8_t ICM_Tx, ICM_Rx;

    ICM_Tx = reg|0x80;
    rt_spi_send_then_recv(spi_dev, &ICM_Tx, 1, &ICM_Rx, 1);

    return ICM_Rx;
}

static uint8_t icm20602_read_buffer(uint8_t reg, void *buffer, uint8_t len , struct  rt_spi_device *spi_dev)
{
    static uint8_t ICM_Tx_buff[14] = {0xff};

    ICM_Tx_buff[0] = reg|0x80;

    rt_spi_send_then_recv(spi_dev, ICM_Tx_buff, 1, buffer, len);

    return 0;
}

static float _accel_scale = 0, _gyro_scale = 0;

//ICM20_ACCEL_FS_2G
//ICM20_ACCEL_FS_4G
//ICM20_ACCEL_FS_8G
//ICM20_ACCEL_FS_16G
static uint8_t icm20602_set_accel_fullscale(uint8_t fs , struct  rt_spi_device *spi_dev)
{
    switch(fs)
    {
    case ICM20_ACCEL_FS_2G:
        _accel_scale = 2.0f/32768.0f;
        break;
    case ICM20_ACCEL_FS_4G:
        _accel_scale = 4.0f/32768.0f;
        break;
    case ICM20_ACCEL_FS_8G:
        _accel_scale = 8.0f/32768.0f;
        break;
    case ICM20_ACCEL_FS_16G:
        _accel_scale = 16.0f/32768.0f;
        break;
    default:
        fs = ICM20_ACCEL_FS_8G;
        _accel_scale = 8.0f/32768.0f;
        break;

    }
    _accel_scale *= GRAVITY_MSS;
    return icm20602_write_reg(ICM20_ACCEL_CONFIG,fs,spi_dev);
}

//ICM20_GYRO_FS_250
//ICM20_GYRO_FS_500
//ICM20_GYRO_FS_1000
//ICM20_GYRO_FS_2000
static uint8_t icm20602_set_gyro_fullscale(uint8_t fs , struct  rt_spi_device *spi_dev)
{
    switch(fs)
    {
    case ICM20_GYRO_FS_250DPS:
        _gyro_scale = 250.0f/32768.0f;	//32767/250
        break;
    case ICM20_GYRO_FS_500DPS:
        _gyro_scale = 500.0f/32768.0f;
        break;
    case ICM20_GYRO_FS_1000DPS:
        _gyro_scale = 1000.0f/32768.0f;
        break;
    case ICM20_GYRO_FS_2000DPS:
        _gyro_scale = 2000.0f/32768.0f;
        break;
    default:
        fs = ICM20_GYRO_FS_2000DPS;
        _gyro_scale = 2000.0f/32768.0f;
        break;

    }
    _gyro_scale *= DEG_TO_RAD;
    return icm20602_write_reg(ICM20_GYRO_CONFIG,fs,spi_dev);

}

int icm20602_get_accel_IMU1(float *accel)
{
    uint8_t buf[6];

    if(icm20602_read_buffer(ICM20_ACCEL_XOUT_H,buf,6,spi_dev_IMU1))
    {
        return 1;
    }

    accel[0] = ((int16_t)((buf[0] << 8) + buf[1])) * _accel_scale;
    accel[1] = ((int16_t)((buf[2] << 8) + buf[3])) * _accel_scale;
    accel[2] = ((int16_t)((buf[4] << 8) + buf[5])) * _accel_scale;
    return 0;
}

int icm20602_get_accel_IMU2(float *accel)
{
    uint8_t buf[6];

    if(icm20602_read_buffer(ICM20_ACCEL_XOUT_H,buf,6,spi_dev_IMU2))
    {
        return 1;
    }

    accel[0] = ((int16_t)((buf[0] << 8) + buf[1])) * _accel_scale;
    accel[1] = ((int16_t)((buf[2] << 8) + buf[3])) * _accel_scale;
    accel[2] = ((int16_t)((buf[4] << 8) + buf[5])) * _accel_scale;
    return 0;
}

int icm20602_get_gyro_IMU1(float *gyro)
{
    uint8_t buf[6];

    if(icm20602_read_buffer(ICM20_GYRO_XOUT_H,buf,6,spi_dev_IMU1))
    {
        return 1;
    }
    
    gyro[0] = ((int16_t)((buf[0] << 8) + buf[1])) * _gyro_scale;
    gyro[1] = ((int16_t)((buf[2] << 8) + buf[3])) * _gyro_scale;
    gyro[2] = ((int16_t)((buf[4] << 8) + buf[5])) * _gyro_scale;
    return 0;
}

int icm20602_get_gyro_IMU2(float *gyro)
{
    uint8_t buf[6];

    if(icm20602_read_buffer(ICM20_GYRO_XOUT_H,buf,6,spi_dev_IMU2))
    {
        return 1;
    }
    
    gyro[0] = ((int16_t)((buf[0] << 8) + buf[1])) * _gyro_scale;
    gyro[1] = ((int16_t)((buf[2] << 8) + buf[3])) * _gyro_scale;
    gyro[2] = ((int16_t)((buf[4] << 8) + buf[5])) * _gyro_scale;
    return 0;
}

// 单位摄氏度
int icm20602_get_temper_IMU1(float *temper)
{
    int16_t temp_adc;
    uint8_t buf[2];

    if(icm20602_read_buffer(ICM20_TEMP_OUT_H,buf,2,spi_dev_IMU1))
    {
        return 1;
    }

    temp_adc = (((int16_t)buf[0])<<8)+buf[1];

    temper[0] = (25.0f + (float)temp_adc/326.8f);
    return 0;
}

int icm20602_get_temper_IMU2(float *temper)
{
    int16_t temp_adc;
    uint8_t buf[2];

    if(icm20602_read_buffer(ICM20_TEMP_OUT_H,buf,2,spi_dev_IMU2))
    {
        return 1;
    }

    temp_adc = (((int16_t)buf[0])<<8)+buf[1];

    temper[0] = (25.0f + (float)temp_adc/326.8f);
    return 0;
}

int icm20602_Reginit(struct  rt_spi_device *spi_dev)
{
    rt_uint8_t id;
    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x80 , spi_dev); //复位，复位后位0x41,睡眠模式
    rt_thread_mdelay(10);
    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x01 , spi_dev); //关闭睡眠，自动选择时钟
    rt_thread_mdelay(10);

    id = icm20602_read_reg(ICM20_WHO_AM_I , spi_dev); //读取ID
    if (id != 0x12)
    {
        rt_kprintf("icm init failed! error id is %x !\n", id);
        return 1;
    }

    icm20602_write_reg(ICM20_PWR_MGMT_2, 0x00 , spi_dev);
    icm20602_write_reg(ICM20_SMPLRT_DIV, 0 , spi_dev);                                  //分频数=为0+1，数据输出速率为内部采样速率
    icm20602_write_reg(ICM20_CONFIG, DLPF_BW_41 , spi_dev);                             // GYRO低通滤波设置  1k rate
    icm20602_write_reg(ICM20_ACCEL_CONFIG2, ACCEL_AVER_4 | ACCEL_DLPF_BW_44 , spi_dev); // ACCEL低通滤波设置

    //设置量程
    icm20602_set_accel_fullscale(ICM20_ACCEL_FS_16G , spi_dev);   //±8g
    icm20602_set_gyro_fullscale(ICM20_GYRO_FS_2000DPS , spi_dev); //±2000dps

    icm20602_write_reg(ICM20_LP_MODE_CFG, 0x00 , spi_dev); //关闭低功耗
    icm20602_write_reg(ICM20_FIFO_EN, 0x00 , spi_dev);     //关闭FIFO

    icm20602_write_reg(ICM20_INT_PIN_CFG, (uint8_t)(1 << 4) , spi_dev); // INT输出设置
    icm20602_write_reg(ICM20_INT_ENABLE, (uint8_t)(1 << 0) , spi_dev);  // INT输出使能

    rt_thread_mdelay(10);

    return 0;
}

// 初始化SPI和ICM20602芯片
void ICM_init()
{
    rt_int8_t flag = 0 ;

    // 配置 SPI 设备
    spi_ICM20602_init();
    
    if(icm20602_Reginit(spi_dev_IMU1))
    {
        rt_kprintf("IMU1 error");
        flag ++ ;
    }

    if(icm20602_Reginit(spi_dev_IMU2))
    {
        rt_kprintf("IMU2 error");
        flag ++ ;
    }

    if(flag == 2)
        while(1);

}
