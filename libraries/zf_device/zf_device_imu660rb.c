/*********************************************************************************************************************
* CYT2BL3 Opensourec Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 文件名称          zf_device_imu660rb
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
* 修改记录
* 日期              作者                备注
* 2025-12-13       github-copilot      移植版本（SPI，引脚同 imu660ra）
*********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_device_config.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

#include "zf_device_imu660rb.h"

int16 imu660rb_gyro_x = 0, imu660rb_gyro_y = 0, imu660rb_gyro_z = 0;
int16 imu660rb_acc_x  = 0, imu660rb_acc_y  = 0, imu660rb_acc_z  = 0;
float imu660rb_transition_factor[2] = {4098.0f, 14.3f}; // 与官方 TC264 版一致的比例

// SPI 读写封装
static void imu660rb_write_register(uint8 reg, uint8 data)
{
    IMU660RB_CS(0);
    spi_write_8bit_register(IMU660RB_SPI, reg | IMU660RB_SPI_W, data);
    IMU660RB_CS(1);
}

static uint8 imu660rb_read_register(uint8 reg)
{
    uint8 data = 0;
    IMU660RB_CS(0);
    data = spi_read_8bit_register(IMU660RB_SPI, reg | IMU660RB_SPI_R);
    IMU660RB_CS(1);
    return data;
}

static void imu660rb_read_registers(uint8 reg, uint8 *data, uint32 len)
{
    IMU660RB_CS(0);
    spi_read_8bit_registers(IMU660RB_SPI, reg | IMU660RB_SPI_R, data, len);
    IMU660RB_CS(1);
}

static uint8 imu660rb_self_check(void)
{
    uint8 dat = 0;
    uint16 timeout_count = 0;
    do
    {
        if(IMU660RB_TIMEOUT_COUNT < timeout_count++)
        {
            return 1;
        }
        dat = imu660rb_read_register(IMU660RB_WHO_AM_I);
        system_delay_ms(1);
    } while(0x6B != dat); // WHO_AM_I 期望值
    return 0;
}

void imu660rb_get_acc(void)
{
    uint8 dat[6];
    imu660rb_read_registers(IMU660RB_OUTX_L_A, dat, 6);
    imu660rb_acc_x = (int16)(((uint16)dat[1] << 8) | dat[0]);
    imu660rb_acc_y = (int16)(((uint16)dat[3] << 8) | dat[2]);
    imu660rb_acc_z = (int16)(((uint16)dat[5] << 8) | dat[4]);
}

void imu660rb_get_gyro(void)
{
    uint8 dat[6];
    imu660rb_read_registers(IMU660RB_OUTX_L_G, dat, 6);
    imu660rb_gyro_x = (int16)(((uint16)dat[1] << 8) | dat[0]);
    imu660rb_gyro_y = (int16)(((uint16)dat[3] << 8) | dat[2]);
    imu660rb_gyro_z = (int16)(((uint16)dat[5] << 8) | dat[4]);
}

uint8 imu660rb_init(void)
{
    uint8 return_state = 0;
    system_delay_ms(20);

    spi_init(IMU660RB_SPI, SPI_MODE0, IMU660RB_SPI_SPEED, IMU660RB_SPC_PIN, IMU660RB_SDI_PIN, IMU660RB_SDO_PIN, SPI_CS_NULL);
    gpio_init(IMU660RB_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    imu660rb_read_register(IMU660RB_WHO_AM_I); // dummy read to switch to SPI

    if(imu660rb_self_check())
    {
        zf_log(0, "imu660rb self check error.");
        return_state = 1;
        return return_state;
    }

    // 软复位
    imu660rb_write_register(IMU660RB_CTRL3_C, 0x01);
    system_delay_ms(10);

    // ACC: 104Hz, +/-8g (0x58 -> ODR 104Hz, FS 8g)
    imu660rb_write_register(IMU660RB_CTRL1_XL, 0x58);
    // GYRO: 104Hz, +/-2000dps (0x5C -> ODR 104Hz, FS 2000dps)
    imu660rb_write_register(IMU660RB_CTRL2_G, 0x5C);
    // CTRL3_C: BDU=1, auto-increment=1, little endian
    imu660rb_write_register(IMU660RB_CTRL3_C, 0x44);
    // CTRL4_C: enable gyro sleep disable, i2c disable=0 (默认)
    imu660rb_write_register(IMU660RB_CTRL4_C, 0x00);
    // CTRL6_C: high-performance enable
    imu660rb_write_register(IMU660RB_CTRL6_C, 0x10);

    return return_state;
}
