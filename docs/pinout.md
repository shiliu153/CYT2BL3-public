# 引脚与片上资源

本文列出本工程占用的引脚与片上外设资源。**所有数值直接取自源码**，未经过实车核对。

引脚名形如 `P00_2`，端口与引脚号之间用下划线分隔。带复用功能的引脚在名称中体现，
例如 `TCPWM_CH13_P00_3` 表示该引脚被配置为 TCPWM 通道 13 的输出。

---

## 1. 运动控制

### 1.1 电机

| 功能 | 宏 | 引脚 / 资源 | 说明 |
|---|---|---|---|
| 左电机方向 | `MOTOR_L_DIR_PIN` | `P00_2` | 推挽输出，初值低 |
| 左电机 PWM | `MOTOR_L_PWM_PIN` | `TCPWM_CH13_P00_3` | 17 kHz，占空比量程 10000 |
| 右电机方向 | `MOTOR_R_DIR_PIN` | `P18_6` | 推挽输出，**初值高**（与左电机相反） |
| 右电机 PWM | `MOTOR_R_PWM_PIN` | `TCPWM_CH50_P18_7` | 17 kHz |

定义位置：`main_cm4.c:12-15`、`motor_init()` 在 `main_cm4.c:1416`。

左右方向引脚初值相反，对应两个电机在车体上的镜像安装。

### 1.2 舵机

| 功能 | 宏 | 引脚 / 资源 |
|---|---|---|
| 舵机 PWM | `SERVO_PWM_PIN` | `TCPWM_CH52_P18_5` |
| PWM 频率 | `SERVO_PWM_FREQ` | 300 Hz |

定义位置：`main_cm4.c:18-19`。duty 量程与标定值见
[Flash 参数](flash-params.md#4-完整参数表)。

### 1.3 编码器

| 功能 | 宏 | 资源 |
|---|---|---|
| 左编码器计数器 | `ENCODER_L` | `TC_CH09_ENCODER` |
| 左编码器 A 相 | `ENCODER_L_A_PIN` | `TC_CH09_ENCODER_CH1_P05_0` |
| 左编码器 B 相 | `ENCODER_L_B_PIN` | `TC_CH09_ENCODER_CH2_P05_1` |
| 右编码器计数器 | `ENCODER_R` | `TC_CH07_ENCODER` |
| 右编码器 A 相 | `ENCODER_R_A_PIN` | `TC_CH07_ENCODER_CH1_P02_0` |
| 右编码器 B 相 | `ENCODER_R_B_PIN` | `TC_CH07_ENCODER_CH2_P02_1` |

定义位置：`main_cm4.c:26-31`，初始化在 `main_cm4.c:374-375`（正交模式
`encoder_quad_init()`）。

左右编码器的读取符号不同：`cm4_isr.c:125-127` 中左轮取反、右轮不取反，
使「正数 = 前进」对两侧一致。

---

## 2. 传感器

### 2.1 摄像头：总钻风 MT9V03X

| 功能 | 宏 | 引脚 |
|---|---|---|
| IIC 时钟（配置用） | `MT9V03X_COF_IIC_SCL` | `P11_2` |
| IIC 数据（配置用） | `MT9V03X_COF_IIC_SDA` | `P11_1` |
| 像素时钟 | `MT9V03X_PCLK_PIN` | `P08_1` |
| 场中断 | `MT9V03X_VSYNC_PIN` | `P08_0` |
| 数据总线 | `MT9V03X_DATA_PIN` | `P06_0` ~ `P06_6`（连续 7 位） |

定义位置：`libraries/zf_device/zf_device_mt9v03x.h`。

> **本工程对上游的改动**：SCL 与 SDA 的引脚相对逐飞原始库对调过
> （原始为 SCL = `P11_1`、SDA = `P11_2`）。见 `NOTICE.md` §1.1。

采集参数（同文件）：

| 宏 | 值 | 含义 |
|---|---:|---|
| `MT9V03X_W` / `MT9V03X_H` | 180 / 120 | 图像宽高 |
| `MT9V03X_EXP_TIME_DEF` | 400 | 曝光时间上限 |
| `MT9V03X_FPS_DEF` | 100 | 帧率上限 |
| `MT9V03X_GAIN_DEF` | 32 | 图像增益（范围 16–64） |
| `MT9V03X_AUTO_EXP_DEF` | 0 | 自动曝光关闭 |
| `MT9V03X_LR_OFFSET_DEF` / `_UD_OFFSET_DEF` | 0 | 图像偏移 |

### 2.2 IMU：IMU660RB

| 功能 | 宏 | 引脚 / 资源 |
|---|---|---|
| SPI 外设 | `IMU660RB_SPI` | `SPI_4` |
| SPI 速率 | `IMU660RB_SPI_SPEED` | 10 MHz |
| 时钟 | `IMU660RB_SPC_PIN` | `SPI4_CLK_P14_2` |
| MOSI | `IMU660RB_SDI_PIN` | `SPI4_MOSI_P14_1` |
| MISO | `IMU660RB_SDO_PIN` | `SPI4_MISO_P14_0` |
| 片选 | `IMU660RB_CS_PIN` | `P18_0` |
| 器件地址 | `IMU660RB_DEV_ADDR` | `0x6B` |

定义位置：`libraries/zf_device/zf_device_imu660rb.h:7-22`。

本工程只使用 Z 轴角速度（`imu660rb_gyro_z`），经 `imu660rb_gyro_transition()`
换算为 dps。加速度计仅在 `feature/adaptive-find-lines` 分支的姿态解算中使用。

---

## 3. 人机交互

### 3.1 按键与指示灯

| 功能 | 宏 | 引脚 |
|---|---|---|
| 按键 1 | `KEY1` | `P11_0` |
| 按键 2 | `KEY2` | `P22_0` |
| 按键 3 | `KEY3` | `P23_3` |
| 按键 4 | `KEY4` | `P23_4` |
| 指示灯 | `LED1` | `P23_7` |

定义位置：`main_cm4.c:34-38`。

`LED1` 在本工程中承担两处用途：

1. **摄像头初始化失败时翻转**（`main_cm4.c:385`），提示系统卡在启动阶段
2. **运行中丢线时翻转并退出**（`main_cm4.c:663-665`）

### 3.2 显示：IPS200

本工程使用 **SPI 模式**（`#define IPS200_TYPE (IPS200_TYPE_SPI)`，`main_cm4.c:96`）。

| 功能 | 宏 | 引脚 / 资源 |
|---|---|---|
| SPI 外设 | `IPS200_SPI` | `SPI_3` |
| SPI 速率 | `IPS200_SPI_SPEED` | 30 MHz |
| 时钟 | `IPS200_SCL_PIN_SPI` | `SPI3_CLK_P13_2` |
| MOSI | `IPS200_SDA_PIN_SPI` | `SPI3_MOSI_P13_1` |
| MISO | `IPS200_SDA_IN_PIN_SPI` | `SPI_MISO_NULL`（IPS 无 MISO） |
| 数据/命令 | `IPS200_DC_PIN_SPI` | `P12_1` |
| 片选 | `IPS200_CS_PIN_SPI` | `P13_3` |
| 背光 | `IPS200_BLk_PIN_SPI` | `P13_0` |

定义位置：`libraries/zf_device/zf_device_ips200.h`。
该驱动同时定义了并口（双排针）模式的一套引脚，本工程未使用。

屏幕分辨率 180 × 320。工程用它显示菜单、参数与图像调试信息
（二值图、左右边界、中线、巡线轨迹点）。

---

## 4. 调试串口

| 功能 | 宏 | 值 |
|---|---|---|
| 串口外设 | `DEBUG_UART_INDEX` | `UART_0` |
| 波特率 | `DEBUG_UART_BAUDRATE` | 115200 |
| TX | `DEBUG_UART_TX_PIN` | `UART0_TX_P00_1` |
| RX | `DEBUG_UART_RX_PIN` | `UART0_RX_P00_0` |
| 接收中断 | `DEBUG_UART_USE_INTERRUPT` | 1（启用） |
| 环形缓冲区 | `DEBUG_RING_BUFFER_LEN` | 64 字节 |

定义位置：`libraries/zf_common/zf_common_debug.h`。

---

## 5. 定时器资源

PIT（Periodic Interval Timer）通道枚举与周期能力（`libraries/zf_driver/zf_driver_pit.h`）：

| 通道范围 | 支持的中断周期 |
|---|---|
| `PIT_CH0` – `PIT_CH3` | 10 µs – 536 s |
| `PIT_CH10` – `PIT_CH21` | 10 µs – 1 s |

本工程占用情况：

| 通道 | 周期 | 用途 |
|---|---:|---|
| `PIT_CH0` | 10,000 µs（100 Hz） | 速度环 |
| `PIT_CH1` | 7,000 µs（≈143 Hz） | 转向环 |
| 其余 13 个通道 | — | 空函数体占位（`cm4_isr.c`） |

初始化：`pit_us_init(PIT_CH0, SPEED_CONTROL_PERIOD)` / `pit_us_init(PIT_CH1, 7000)`
（`main_cm4.c:412-413`）。

---

## 6. Flash

| 项 | 值 |
|---|---|
| 用途 | 保存 26 个可调参数 |
| 页索引 | 0 |
| 驱动 | `libraries/zf_driver/zf_driver_flash.c` |
| 初始化 | `flash_init()`（`main_cm4.c:405`） |

详见 [Flash 参数](flash-params.md)。

---

## 7. 资源冲突检查清单

改动引脚或外设时，需要确认以下几点（本工程当前满足）：

- [ ] 每个 `TCPWM` 通道只被一个功能占用（电机左/右、舵机共 3 个通道）
- [ ] 每个 `TC`（计数器）通道只被一个编码器占用（`TC_CH09`、`TC_CH07`）
- [ ] SPI 外设不冲突（IMU 用 `SPI_4`，屏幕用 `SPI_3`）
- [ ] 摄像头数据总线 `P06_0`–`P06_6` 连续且未被其他功能占用
- [ ] `PIT_CH0` / `PIT_CH1` 的周期与各自的闭环带宽匹配

---

## 8. 相关文档

- [系统架构](architecture.md)
- [控制链路与调参](control-and-tuning.md)
- [Flash 参数](flash-params.md)
