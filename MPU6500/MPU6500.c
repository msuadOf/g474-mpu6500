#include <stdio.h>
#include <stdint.h>
#include "MPU6500.h"
/* 选择使用SPI还是I2C进行通讯捏，当果然是SPI腻 */
#define MPU6500_USING_SPI
// #define MPU6500_USING_I2C
extern void mpu6500_delay_ms(int ms);
#ifdef MPU6500_USING_SPI

extern void hal_spi_init();

extern void hal_spi_read(unsigned char addr, unsigned char *buffer, int len);

extern void hal_spi_write(unsigned char addr, unsigned char *buffer, int len);

#endif

#ifdef MPU6500_USING_I2C
#include "STC32G_Soft_I2C.h"
void hal_i2c_init()
{
    P2_SPEED_HIGH(GPIO_Pin_2 | GPIO_Pin_3);
}
void hal_i2c_read(unsigned char addr, unsigned char *buffer, int len)
{
}

void hal_i2c_write(unsigned char addr, unsigned char *buffer, int len)
{
    SI2C_WriteNbyte(addr, buffer, len);
}
#endif
void MPU6500_Peripheral_Init()
{
#ifdef MPU6500_USING_I2C
    hal_i2c_init();
#endif
#ifdef MPU6500_USING_SPI
    hal_spi_init();
#endif
}
void MPU6500_Read(unsigned char addr, unsigned char *buffer, int len)
{
#ifdef MPU6500_USING_I2C
    hal_i2c_read(addr, buffer, len);
#endif
#ifdef MPU6500_USING_SPI
    hal_spi_read(addr, buffer, len);
#endif
}

void MPU6500_Write(unsigned char addr, unsigned char *buffer, int len)
{
#ifdef MPU6500_USING_I2C
    hal_i2c_write(addr, buffer, len);
#endif
#ifdef MPU6500_USING_SPI
    hal_spi_write(addr, buffer, len);
#endif
}
/**
 * *******************************************
 *
 *  兼容层捏 end
 *
 * *******************************************
 * */

/* 定义MPU6500初始化函数 */
void MPU6500_Init()
{
    unsigned char reg_buffer;

    MPU6500_Peripheral_Init();

    // /* 设置陀螺仪采样率为1KHz */
    // reg_buffer = 0x00; //分频系数为0
    // MPU6500_Write(MPU6500_SMPLRT_DIV, &reg_buffer, 1);

    // /* 设置加速度计采样率为1KHz */
    // reg_buffer = 0x00; //分频系数为0
    // MPU6500_Write(MPU6500_ACCEL_CONFIG, &reg_buffer, 1);

    /* 设置陀螺仪自检 */
    reg_buffer = 0x80;
    MPU6500_Write(MPU6500_SELF_TEST_X_GYRO, &reg_buffer, 1);
    // 等待100ms，让陀螺仪完成自检
    mpu6500_delay_ms(100);

    /* 关闭陀螺仪自检 */
    reg_buffer = 0x00;
    MPU6500_Write(MPU6500_SELF_TEST_X_GYRO, &reg_buffer, 1);

    /* 打开陀螺仪和加速度计 */
    reg_buffer = 0x00;
    MPU6500_Write(MPU6500_PWR_MGMT_1, &reg_buffer, 1);

    reg_buffer = MPU6500_GYRO_FS_SEL_500dps;
    MPU6500_Write(MPU6500_GYRO_CONFIG, &reg_buffer, 1);

    reg_buffer = MPU6500_ACCEL_FS_SEL_4g;
    MPU6500_Write(MPU6500_ACCEL_CONFIG, &reg_buffer, 1);

    reg_buffer = 0;
    do
    {
        MPU6500_Read(MPU6500_WHO_AM_I, &reg_buffer, 1);
    } while (reg_buffer != 0x70);
}

/* 定义MPU6500六轴数据获取函数 */
float Get_16Bit_Data(uint8_t addr_h_data, uint8_t addr_l_data)
{

    static short data;

    data = (addr_h_data << 8) | addr_l_data;
    return data;
}

void MPU6500_get_buffer(float *gyro_buffer, float *acc_buffer)
{
    static float gyroLast[3] = {0, 0, 0}; // 上一次的陀螺仪数据
    static float accLast[3] = {0, 0, 0};  // 上一次的加速度计数据

    unsigned char buf[14];
    float alphaGyro = 0.5f; // 陀螺仪数据的滤波系数
    float alphaAcc = 0.5f;  // 加速度计数据的滤波系数

    /* 读取陀螺仪和加速度计数据 */
    MPU6500_Read(MPU6500_ACCEL_XOUT_H, buf, (3 + 1 + 3) * 2); // 3个gyro轴数据+1个temp数据+3个acc数据

    /* 解析陀螺仪数据，使用互补滤波平滑数据 */

    gyro_buffer[0] = (float)(((short)buf[8] << 8) | buf[9]) / 32768.0f * 500.0f;
    gyro_buffer[1] = (float)(((short)buf[10] << 8) | buf[11]) / 32768.0f * 500.0f;
    gyro_buffer[2] = (float)(((short)buf[12] << 8) | buf[13]) / 32768.0f * 500.0f;
    gyro_buffer[0] = alphaGyro * gyroLast[0] + (1 - alphaGyro) * gyro_buffer[0];
    gyro_buffer[1] = alphaGyro * gyroLast[1] + (1 - alphaGyro) * gyro_buffer[1];
    gyro_buffer[2] = alphaGyro * gyroLast[2] + (1 - alphaGyro) * gyro_buffer[2];
    gyroLast[0] = gyro_buffer[0];
    gyroLast[1] = gyro_buffer[1];
    gyroLast[2] = gyro_buffer[2];

    /* 解析加速度计数据，使用互补滤波平滑数据 */

    acc_buffer[0] = Get_16Bit_Data(buf[0] ,buf[1]) / 32768.0f * 4.0f;
    acc_buffer[1] = Get_16Bit_Data(buf[2] ,buf[3]) / 32768.0f * 4.0f;
    acc_buffer[2] = Get_16Bit_Data(buf[4] ,buf[5]) / 32768.0f * 4.0f;
    acc_buffer[0] = alphaAcc * accLast[0] + (1 - alphaAcc) * acc_buffer[0];
    acc_buffer[1] = alphaAcc * accLast[1] + (1 - alphaAcc) * acc_buffer[1];
    acc_buffer[2] = alphaAcc * accLast[2] + (1 - alphaAcc) * acc_buffer[2];
    accLast[0] = acc_buffer[0];
    accLast[1] = acc_buffer[1];
    accLast[2] = acc_buffer[2];
}

// int main()
// {
//     float gyro_buffer[3], acc_buffer[3];

//     /* 初始化MPU6500 */
//     mpu6500_init();

//     while (1)
//     {
//         /* 获取陀螺仪和加速度计数据 */
//         mpu6500_get_buffer(gyro_buffer, acc_buffer);

//         /* 打印数据 */
//         printf("Gyro: %.2f, %.2f, %.2f, Acc: %.2f, %.2f, %.2f\n", gyro_buffer[0], gyro_buffer[1], gyro_buffer[2], acc_buffer[0], acc_buffer[1], acc_buffer[2]);

//         /* 延时一段时间 */
//         delay_ms(10);
//     }

//     return 0;
// }
