#ifndef _zf_device_imu660rb_h_
#define _zf_device_imu660rb_h_

#include "zf_common_typedef.h"

//================================================定义 IMU660RB 基本配置================================================
#define IMU660RB_SPI_SPEED            (10 * 1000 * 1000)
#define IMU660RB_SPI                  (SPI_4)
#define IMU660RB_SPC_PIN              (SPI4_CLK_P14_2)
#define IMU660RB_SDI_PIN              (SPI4_MOSI_P14_1)
#define IMU660RB_SDO_PIN              (SPI4_MISO_P14_0)
#define IMU660RB_CS_PIN               (P18_0)
#define IMU660RB_CS(x)                ((x) ? (gpio_high(IMU660RB_CS_PIN)) : (gpio_low(IMU660RB_CS_PIN)))

#define IMU660RB_TIMEOUT_COUNT        (0x00FF)

//================================================定义 IMU660RB 内部地址================================================
#define IMU660RB_DEV_ADDR             (0x6B)
#define IMU660RB_SPI_W                (0x00)
#define IMU660RB_SPI_R                (0x80)

#define IMU660RB_WHO_AM_I             (0x0F)
#define IMU660RB_CTRL1_XL             (0x10)
#define IMU660RB_CTRL2_G              (0x11)
#define IMU660RB_CTRL3_C              (0x12)
#define IMU660RB_CTRL4_C              (0x13)
#define IMU660RB_CTRL6_C              (0x15)
#define IMU660RB_OUTX_L_G             (0x22)
#define IMU660RB_OUTX_L_A             (0x28)

//================================================声明 IMU660RB 全局变量================================================
extern int16 imu660rb_gyro_x, imu660rb_gyro_y, imu660rb_gyro_z;
extern int16 imu660rb_acc_x,  imu660rb_acc_y,  imu660rb_acc_z;
extern float imu660rb_transition_factor[2];
//================================================声明 IMU660RB 全局变量================================================

//================================================声明 IMU660RB 基础函数================================================
void    imu660rb_get_acc            (void);
void    imu660rb_get_gyro           (void);
uint8   imu660rb_init               (void);
//================================================声明 IMU660RB 基础函数================================================

//================================================声明 IMU660RB 拓展函数================================================
#define imu660rb_acc_transition(acc_value)   ((float)(acc_value) / imu660rb_transition_factor[0])
#define imu660rb_gyro_transition(gyro_value) ((float)(gyro_value) / imu660rb_transition_factor[1])
//================================================声明 IMU660RB 拓展函数================================================

#endif
