# 系统架构

本文说明工程的分层依赖、启动时序、前后台分工与中断资源分配。

---

## 1. 分层依赖

工程沿用逐飞库的六层结构。约定是：**上层只依赖下层的公开接口，不反向依赖**。

| 层 | 路径 | 内容 | 归属 |
|---|---|---|---|
| 6 | `project/user/` | 应用：图像处理、元素识别、控制、菜单、参数 | 本工程 |
| 5 | `project/code/` | 用户代码存放位置（本工程未使用，仅保留目录说明） | — |
| 4 | `libraries/zf_components/` | 组件：`seekfree_assistant` 上位机通信协议 | 逐飞 |
| 3 | `libraries/zf_device/` | 设备驱动：MT9V03X 摄像头、IMU660RB、IPS200 屏幕、按键、无线模块 | 逐飞 |
| 2 | `libraries/zf_driver/` | 外设驱动：GPIO、PWM、PIT、UART、SPI、ADC、编码器、Flash、EXIT | 逐飞 |
| 1 | `libraries/zf_common/` | 基础设施：时钟、调试、FIFO、字体、常用函数、中断封装 | 逐飞 |
| 0 | `libraries/sdk/` | 芯片厂商 SDK（TRAVEO™ T2G Sample Driver Library） | 英飞凌 / 赛普拉斯 |

### 1.1 统一入口头

`libraries/zf_common/zf_common_headfile.h` 汇聚第 0–4 层的全部公开头文件，
应用层只需：

```c
#include "zf_common_headfile.h"
```

它同时包含芯片 SDK 的 `cy_project.h` 与 `cy_device_headers.h`，因此寄存器级操作也可以
直接使用。本工程对逐飞库的这一文件做了两处补充：新增 `zf_device_imu660rb.h` 与
`pid.h` 的包含（见 `NOTICE.md` §1.1）。

### 1.2 依赖方向的一个例外

`libraries/zf_common/pid.h` 是本工程新增的文件，放在逐飞库目录下，因为它需要被
`zf_common_headfile.h` 包含，而该目录已经在 IAR 工程的包含路径中。这属于工程便利性的
安排，不改变逻辑分层——`pid.h` 只定义数据结构，不含逻辑。

---

## 2. 启动时序

入口：`project/user/main_cm4.c:365` 的 `int main(void)`。

| 步骤 | 调用 | 说明 |
|---|---|---|
| 1 | `clock_init(SYSTEM_CLOCK_160M)` | 160 MHz。PWM 频率、PIT 周期、串口波特率、摄像头采样节拍最终都落在这个基准上 |
| 2 | `debug_init()` | 调试串口与断言输出 |
| 3 | `gpio_init(LED1, ...)` | 最小的运行状态指示 |
| 4 | `keys_init()` | 按键输入，调试菜单的入口 |
| 5 | `motor_init()` | 方向引脚 + 17 kHz PWM，占空比初值 0 |
| 6 | `encoder_quad_init(ENCODER_L/R, ...)` | 两路正交编码器 |
| 7 | `system_delay_ms(100)` | 等待外设稳定 |
| 8 | `ips200_init(IPS200_TYPE)` | 屏幕初始化 |
| 9 | `while (mt9v03x_init()) { LED 翻转; 显示提示; 延时 500 ms; }` | **摄像头失败则循环重试** |
| 10 | `imu660rb_init()` | IMU 初始化 |
| 11 | `calculate_otsu_threshold(mt9v03x_image)` | 对首帧求一次大津法阈值作为初值 |
| 12 | `pid_init()` × 3 | 转向 15.1 / 0.00 / 1.9；左右速度 16.8 / 2.5 / 0.0 |
| 13 | `flash_init()` + `params_load_from_flash()` | 用上次保存的参数覆盖默认值 |
| 14 | 舵机回中后 `pwm_init(SERVO_PWM_PIN, 300, g_servo_center_duty)` | 先回中再使能，避免上电猛打角 |
| 15 | `pit_us_init(PIT_CH0, 10000)` / `pit_us_init(PIT_CH1, 7000)` | 启动速度环与转向环 |
| 16 | `while (true) { switch (g_menu_state) { ... } system_delay_ms(50); }` | 进入菜单状态机 |

### 2.1 为什么这么排序

- **先 LED、按键，后摄像头**：任何复杂模块失败时，仍然保留最小的可视化与交互手段。
  摄像头初始化失败时正是靠 LED 翻转 + 屏幕提示在现场定位问题。
- **阈值在 PID 之前**：阈值是视觉链路的前置条件，先让它有意义。
- **Flash 参数在 PID 初始化之后**：先用代码默认值初始化（保证结构体各字段有确定值），
  再用 Flash 中的值覆盖。这样即使 Flash 为空或损坏，控制器也是可用的。
- **舵机先回中**：`g_servo_duty` 在 `pwm_init` 之前被赋为中位值，避免上电瞬间输出未定义脉宽。
- **PIT 最后启动**：控制中断一旦运行就会立即操作 PWM，必须等所有状态量都初始化完毕。

---

## 3. 前后台分离

这是本工程结构上最关键的一条设计。

| | 前台 | 后台 |
|---|---|---|
| 载体 | `main()` 的 `while(true)` 循环 | `cm4_isr.c` 中的 PIT 中断 |
| 职责 | 菜单显示、按键交互、图像处理、元素识别 | 速度闭环、转向闭环 |
| 执行方式 | 阻塞式：菜单任务内部可无限循环、可 `system_delay_ms()` | 固定周期、不可阻塞 |
| 周期要求 | 不要求实时 | 速度环 10 ms；转向环 7 ms |

### 3.1 主循环：菜单状态机

```c
while (true) {
    switch (g_menu_state) {
        case MENU_MAIN:                 show_main_menu();                  break;
        case MENU_RUN_MAIN_PROGRAM:     run_main_program_task();           break;
        case MENU_SPEED_DEBUG:          menu_speed_debug_task();           break;
        case MENU_PARAM_DEBUG:          menu_param_debug_task();           break;
        case MENU_THRESHOLD_DEBUG:      menu_threshold_debug_task();       break;
        case MENU_PREVIEW_VARS:         menu_preview_var_task();           break;
        case MENU_SERVO_DEBUG:          menu_servo_debug_task();           break;
        case MENU_IMU_DEBUG:            menu_imu_debug_task();             break;
        case MENU_TRACKING_PARAM_DEBUG: menu_tracking_param_debug_task();  break;
    }
    system_delay_ms(50);
}
```

每个 `menu_*_task()` 内部是「显示 → 读按键 → 改参数 → 必要时保存 → 判定是否退出」的循环。
`MENU_MAIN` 与 `MENU_RUN_MAIN_PROGRAM` 是例外：前者只显示一次主菜单就返回（由外层
`system_delay_ms(50)` 限速刷新），后者是正式运行模式，每轮处理一帧图像。

### 3.2 后台：两个控制中断

| 中断 | 周期 | 频率 | 职责 |
|---|---:|---:|---|
| `pit0_ch0_isr()` | 10,000 µs | 100 Hz | 读编码器 → 左右轮速度 PID → 输出电机 PWM |
| `pit0_ch1_isr()` | 7,000 µs | ≈143 Hz | 读 IMU 角速度 → 转向 PID → 输出舵机 PWM → 计算阿克曼差速 → 更新左右轮目标速度 |

两者**周期不同**是有意的：转向是外环、速度是内环，转向需要更高的带宽以跟上视觉更新；
而速度环的周期直接定义了速度的单位（「每 10 ms 的编码器脉冲数」），改动它会同时改变
所有速度参数的含义。

### 3.3 状态门控

两个中断在执行前都检查 `g_menu_state`：

```c
// pit0_ch0_isr()
if ((g_menu_state == MENU_RUN_MAIN_PROGRAM && !g_run_enable) ||
    (g_menu_state != MENU_RUN_MAIN_PROGRAM && g_menu_state != MENU_SPEED_DEBUG)) {
    pwm_set_duty(MOTOR_L_PWM_PIN, 0);
    pwm_set_duty(MOTOR_R_PWM_PIN, 0);
    /* 清空 PID 状态 */
    encoder_clear_count(ENCODER_L);
    encoder_clear_count(ENCODER_R);
    return;
}
```

```c
// pit0_ch1_isr()
if (g_menu_state == MENU_RUN_MAIN_PROGRAM) {
    if (!g_run_enable) return;
    if (!g_track_lost) steering_control();
    return;
}
if (g_menu_state == MENU_THRESHOLD_DEBUG) {
    steering_control_servo_only();   // 只动舵机、不动电机
    return;
}
```

这样安排有两个用处：

1. **安全**：在菜单里改参数时电机不会转；阈值调试时舵机可以跟随中线验证方向，
   但驱动轮保持静止。
2. **不破坏 PID**：清空的是 `output`、`error`、`last_error`、`pre_last_error`、`integral`
   全套状态。只把输出置 0 是不够的——恢复运行时残留的积分与历史误差会造成输出突跳。

---

## 4. 中断资源

`project/user/cm4_isr.c` 中注册的 PIT 中断共 15 个通道（`PIT_CH0`–`PIT_CH2`、
`PIT_CH10`–`PIT_CH21`），其中本工程真正使用的只有前两个：

| 通道 | ISR | 本工程用途 |
|---|---|---|
| `PIT_CH0` | `pit0_ch0_isr()` | 速度环（10 ms） |
| `PIT_CH1` | `pit0_ch1_isr()` | 转向环（7 ms） |
| `PIT_CH2`、`PIT_CH10`–`PIT_CH21` | 对应 `pit0_chN_isr()` | 空函数体，逐飞库预留的入口占位 |

`cm4_isr.c` 共 46 个函数定义，其中 22 个为空函数体：

- 15 个 PIT 通道中 13 个为空
- 7 个 UART 中断（`uart0_isr`–`uart6_isr`）中，只有被调试串口占用的那个有实现
- 24 个 GPIO 外部中断（`gpio_0_exti_isr`–`gpio_23_exti_isr`）中，只有按键相关有实现

保留这些占位是有必要的：逐飞库的 `pit_us_init()` 等接口会引用这些符号，缺少定义会链接失败。

---

## 5. 数据流总览

```
                  ┌──────────────── 前台（主循环，不要求实时） ────────────────┐
 摄像头 ──帧──▶ 二值化 ─▶ 滤波 ─▶ 八邻域追踪 ─▶ 边界 ─▶ 中线 ─▶ 元素识别与修线
                  │                                              │
                  │                                              ▼
                  │                                       多行加权误差 g_final_error
                  │                                              │
                  └──────────────────────────────────────────────┼──────────────┘
                                                                 │
                  ┌────────────── 后台（PIT 中断，固定周期） ─────▼──────────────┐
   IMU ──角速度────────────────────────────────▶ 转向 PID ─▶ 舵机 PWM
                                                    │
                                                    ▼
                                       舵机 duty ─▶ 转角(rad) ─▶ 阿克曼差速
                                                    │
                                                    ▼
                                           左右轮目标速度 g_target_speed_l/r
                                                    │
   编码器 ──脉冲数────────────────────▶ 速度 PID ◀───┘ ─▶ 电机 PWM
                  └──────────────────────────────────────────────────────────────┘
```

关键点：**视觉（慢）与控制（快）通过少量全局变量解耦**。
`g_final_error`、`g_target_speed_l/r` 等变量由前台写入、后台读取，
前台一帧的处理时间不影响后台的控制节拍。

---

## 6. 相关文档

- [图像处理链路](image-pipeline.md)
- [元素识别](element-recognition.md)
- [控制链路与调参](control-and-tuning.md)
- [Flash 参数](flash-params.md)
- [引脚与资源](pinout.md)
