# CYT2BL3 摄像头循迹小车

基于**逐飞科技 CYT2BL3 开源库 V3.3.3** 的摄像头循迹小车固件，运行在 Infineon **CYT2BL3**
（TRAVEO™ T2G Body High，Cortex-M4）上。

工程实现了从图像到执行机构的完整链路：灰度图像采集与大津法二值化、八邻域边界追踪、
中线提取、双环 PID 控制与阿克曼几何差速，并配有 9 级调试菜单与基于 Flash 的参数持久化。
赛道元素识别（环岛 / 十字 / 停车线 / 斑马线 / 障碍 / 直道）的判定与修线逻辑完整保留在
代码中，但**在本仓库的 `main` 分支上，这些函数的调用处于注释状态**——详见
[第 9 节](#9-分支说明)的分支对照表。

| 指标 | 值 |
|---|---|
| 自研代码 | **2 个文件 / 4,129 行**（`project/user/main_cm4.c` 3,547 行 + `cm4_isr.c` 582 行） |
| 图像规格 | 180 × 120 灰度，大津法动态阈值 |
| 控制周期 | 速度环 100 Hz（10 ms），转向环 ≈143 Hz（7 ms），均由 PIT 中断驱动 |
| 系统时钟 | 160 MHz |
| 构建工具链 | IAR Embedded Workbench for ARM 9.40.1 |
| 分支 | 2 条：`main`（当前开发线）与 `feature/adaptive-find-lines`（含姿态解算与已接入的元素识别） |

> **阅读提示**：本工程的整理范围、排除项与验证方法见 [`docs/PUBLICATION.md`](docs/PUBLICATION.md)；
> 第三方组件与权利归属见 [`NOTICE.md`](NOTICE.md)，其中包含一条**关于芯片厂商 SDK 许可的重要说明**。

---

## 目录

- [1. 硬件平台](#1-硬件平台)
- [2. 系统架构](#2-系统架构)
- [3. 目录结构](#3-目录结构)
- [4. 图像处理链路](#4-图像处理链路)
- [5. 元素识别](#5-元素识别)
- [6. 控制链路](#6-控制链路)
- [7. 参数持久化](#7-参数持久化)
- [8. 调试菜单](#8-调试菜单)
- [9. 分支说明](#9-分支说明)
- [10. 构建与运行](#10-构建与运行)
- [11. 关键参数速查](#11-关键参数速查)
- [12. 已知问题与限制](#12-已知问题与限制)
- [13. 许可](#13-许可)

---

## 1. 硬件平台

| 部件 | 型号 / 规格 | 接口与资源 |
|---|---|---|
| 主控 | Infineon CYT2BL3（Cortex-M4 + Cortex-M0+） | 160 MHz |
| 摄像头 | 总钻风 MT9V03X | 灰度 180 × 120（原始传感器支持到 188 × 120） |
| IMU | IMU660RB | SPI4 @ 10 MHz |
| 电机 × 2 | 直流有刷 + 方向引脚 | PWM 17 kHz；左 `TCPWM_CH13`(P00_3)，右 `TCPWM_CH50`(P18_7) |
| 舵机 | 转向舵机 | PWM 300 Hz，`TCPWM_CH52`(P18_5) |
| 编码器 × 2 | 正交增量式 | 左 `TC_CH09`(P05_0/P05_1)，右 `TC_CH07`(P02_0/P02_1) |
| 显示 | IPS200 | SPI |
| 按键 | KEY1–KEY4 | P11_0 / P22_0 / P23_3 / P23_4 |
| 指示灯 | LED1 | P23_7 |

引脚与资源的完整清单见 [`docs/pinout.md`](docs/pinout.md)。

---

## 2. 系统架构

### 2.1 分层依赖

工程沿用逐飞库的六层结构，上层只依赖下层的公开接口：

```
第 6 层  project/user/                 应用：图像处理 · 元素识别 · 控制 · 菜单 · 参数
第 5 层  project/code/                 用户代码存放位置（本工程未使用）
第 4 层  libraries/zf_components/      组件：seekfree_assistant 上位机通信
第 3 层  libraries/zf_device/          设备驱动：摄像头 · IMU · 屏幕 · 按键 · 无线
第 2 层  libraries/zf_driver/          外设驱动：GPIO · PWM · PIT · UART · SPI · ADC · 编码器 · Flash
第 1 层  libraries/zf_common/          基础设施：时钟 · 调试 · FIFO · 字体 · 中断封装
第 0 层  libraries/sdk/                芯片厂商 SDK（TRAVEO T2G Sample Driver Library）
```

所有层通过统一入口头 `libraries/zf_common/zf_common_headfile.h` 汇聚，应用层只需包含这一个头文件。

### 2.2 前后台分离

这是本工程结构上最关键的一条设计：

| | 前台（主循环） | 后台（PIT 中断） |
|---|---|---|
| 职责 | 菜单显示、按键交互、图像处理、元素识别 | 速度闭环、转向闭环 |
| 执行方式 | 阻塞式循环，可调用 `system_delay_ms()` | 固定周期中断，不可阻塞 |
| 周期 | 不要求实时 | 速度环 10 ms，转向环 7 ms |

主循环是一个以 `g_menu_state` 为状态的 9 分支状态机（`main_cm4.c:415-448`）。
每个菜单任务内部自行显示与等待按键，退出时回到主菜单。真正的实时控制全部放在
`cm4_isr.c` 的两个中断服务函数里，由 PIT 硬件保证周期，因此屏幕刷新或按键等待不会
破坏 PID 的离散化假设。

两个中断服务函数在执行前都会检查 `g_menu_state`：只有在运行模式或对应的调试模式下才输出，
其余状态下把 PWM 置 0 并清空 PID 状态（`cm4_isr.c:44-65`、`163-185`）。这避免了在菜单里
改参数时电机意外转动。

### 2.3 启动时序

`main()`（`main_cm4.c:365`）的顺序体现了「先保证有最小可观测手段，再逐级启动」的思路：

1. `clock_init(SYSTEM_CLOCK_160M)` — 所有时间相关外设的基准
2. `debug_init()` — 调试输出与断言
3. LED、按键、电机、编码器初始化 — 最小的可视化与交互手段
4. `ips200_init()` + 摄像头初始化（**失败则循环重试**，同时翻转 LED 并显示提示）
5. `imu660rb_init()`
6. 对整帧求一次大津法阈值，作为二值化的初值
7. 三个 PID 控制器初始化
8. `flash_init()` + `params_load_from_flash()` — 用上次保存的参数覆盖默认值
9. 舵机先回中（`g_servo_center_duty`）再启动 PWM — 避免上电瞬间猛打角
10. `pit_us_init(PIT_CH0, 10000)` / `pit_us_init(PIT_CH1, 7000)` — 启动两个控制中断
11. 进入菜单状态机主循环

---

## 3. 目录结构

```
.
├── libraries/
│   ├── sdk/                    芯片厂商 SDK（第三方，非开源，见 NOTICE.md §2）
│   ├── zf_common/              公共基础层（逐飞，GPL-3.0）
│   ├── zf_driver/              外设驱动层（逐飞，GPL-3.0）
│   ├── zf_device/              设备驱动层（逐飞，GPL-3.0）
│   ├── zf_components/          组件层（逐飞，GPL-3.0）
│   ├── LICENSE                 逐飞库的 GPL-3.0 正文
│   └── doc/                    逐飞库版本说明与许可声明
├── project/
│   ├── code/                   用户代码存放位置（含目录说明）
│   ├── user/
│   │   ├── main_cm4.c          CM4 主程序（3,547 行）
│   │   └── cm4_isr.c           中断服务函数（582 行）
│   └── iar/
│       ├── cyt2bl3.eww         IAR 工作区
│       ├── project_config/     IAR 工程配置（.ewp/.ewd/.ewt/.ewx）
│       └── icf/                链接脚本
├── docs/                       本工程的说明文档
├── tools/                      发布前自检脚本
├── LICENSE                     GPL-3.0 全文
└── NOTICE.md                   第三方归属与权利披露
```

`project/iar/Debug_m4/`（构建产物）与 `project/iar/settings/`（IAR 个人会话状态）**不在仓库中**，
原因见 [`docs/PUBLICATION.md`](docs/PUBLICATION.md)。

---

## 4. 图像处理链路

一帧图像的处理入口是 `image_process(threshold)`（`main_cm4.c:1952`），流程如下。

### 4.1 动态阈值（大津法）

大津法把灰度直方图切成前景与背景两类，取使**类间方差最大**的阈值。

工程采用三个策略控制它的实时开销：

| 策略 | 取值 | 原因 |
|---|---|---|
| 降采样步长 | 2 | 隔点统计直方图，统计量降为 1/4 |
| 更新间隔 | 每 6 帧一次 | 逐帧更新会让阈值抖动，反而降低二值图稳定性 |
| 初值 | 上电时对整帧求一次 | 避免开头若干帧使用无意义的默认阈值 |

阈值在调试模式下可手动覆盖，运行模式下自动更新（`OTSU_UPDATE_INTERVAL_RUN` /
`OTSU_DOWNSAMPLE_STEP_RUN` 等宏，`main_cm4.c:136-153`）。

### 4.2 二值化与定向滤波

```c
bin_image[i][j] = (src[j] >= threshold) ? 255 : 0;
```

随后做一次噪声滤波，但有三个限制（`main_cm4.c:156-172`）：

- 只在第 1–40 行执行——这是对巡线最关键的行区间，远处细节不做过度平滑
- 每 3 帧执行一次，而不是每帧
- 可通过 `IMAGE_FILTER_ENABLE` 整体关闭

二值图只在**阈值调试菜单**下才会额外写入 `g_image_binary` 供屏幕显示，
运行模式下不产生这份拷贝——这是一处有意的开销削减。

### 4.3 八邻域边界追踪

与逐行扫描相比，八邻域追踪沿边界行走，能抵抗局部断裂，并天然产出可用于元素判别的
**方向序列**。核心实现是 `search_eight_neighbor()`（`main_cm4.c:1543`）。

起始点由 `get_start_points()` 在第 90 行（`MT9V03X_H - 30`）从图像中心向两侧寻找
「白→黑」跳变确定。

追踪过程维护：

| 数据 | 类型 | 作用 |
|---|---|---|
| `points_l/points_r` | `uint16[1024][2]` | 左/右边界轨迹点 |
| `dir_l/dir_r` | `int8[1024]` | 每一步的移动方向（0–7） |
| `l_dir_sum/r_dir_sum` | `uint8[9]` | 方向直方图，供元素判别复用 |

左右两侧使用**方向相反的种子表**（左顺时针、右逆时针），因此两侧都会向图像上方行走。

两个工程细节值得说明：

- **单边冻结**：当某一侧找不到候选点时冻结该侧，让另一侧继续爬线（`main_cm4.c:1573-1598`）。
  这样不会因为一侧短暂丢失边界而中止整帧的追踪。
- **接近顶部时停止记录方向**：`center_point_l[1] < 5` 时不再累加方向直方图
  （`main_cm4.c:1627-1654`），避免图像顶端的噪声污染元素判据。

追踪步数上限为 1,500（由调用方传入 `break_flag`），点集容量 1024。超过容量即停止，
保证不会越界写。

### 4.4 失败兜底：逐行扫描

视觉算法在反光、阴影、断边时不可能 100% 成功。当八邻域追踪失败时，
`fallback_scanline()`（`main_cm4.c:1896`）从图像底部向上逐行寻找边界：

1. 以图像底行中心为起点；若该点为黑，则在 1/4 与 3/4 处试探，选一个白色的位置作为起点
2. 每行以「白→黑→黑」的三像素模式确定左右边界
3. 用本行中点作为下一行（更靠上的一行）的搜索起点

兜底策略的目标不是「最优」，而是「有结果」——即使精度下降，也要给出一条连续中心线，
使控制层不至于因为几帧识别失败而完全失控。

### 4.5 中线生成与提前相遇补偿

边界提取完成后（`main_cm4.c:2071-2094`）：

- 一般情形：`中线 = (左边界 + 右边界) / 2`
- 单侧丢线：用赛道宽度模型补出缺失的一侧。宽度按行号在顶部 60 px 与底部 180 px 之间线性插值
- 两侧同时丢线：沿用上一行的中线

另外，当左右边界在图像中途提前相遇（说明赛道偏向一侧、前方是不可通行区域），
代码会在相遇行以上强行把中线推到画面边缘（`forced_mid_after_meet`，
`main_cm4.c:2030-2068`），使车头继续朝赛道内侧转。

---

## 5. 元素识别

> **启用状态先说清楚**：本节描述的元素判定与修线逻辑在代码中是完整的，但在
> **`main` 分支上，`image_process()` 内的调用块被注释掉了**（`main_cm4.c:2100-2112`）——
> 也就是说 `main` 分支的图像链路只做纯中线提取，不执行元素识别。
> **`feature/adaptive-find-lines` 分支上这些调用是开启的**（对应 `*_ported` 系列函数）。
> 两分支的完整对照见[第 9 节](#9-分支说明)。

### 5.1 设计：判定与处理分离

元素识别不是一个大函数，而是每帧按固定顺序执行的一组步骤，并且严格区分两类职责：

| 职责 | 函数 | 作用 |
|---|---|---|
| **判定** | `element_test()`、`stop_line()`、`judge_bar()`、`straight_judge()` | 只更新标志位与阶段变量，不改边界 |
| **处理** | `element_handle()` | 按标志位修补 `left_border` / `right_border`，再重算中线 |

这样划分的好处是：判定逻辑可以独立于修线策略调试，屏幕上也能分别观察「识别到了什么」
与「修线修成什么样」。

`main` 分支中 `image_process()` 末尾被注释掉的调用块（`main_cm4.c:2100-2112`）原本是：

```c
// uint32 primask = interrupt_global_disable();
// g_encoder_delta_accum = 0;
// interrupt_global_enable(primask);

// (void)stop_line();
// (void)cross_delay_judge();
// element_test();
// (void)straight_judge();
// (void)straight_judge_2();
// judge_bar();

// if (flag_stop || flag_bar || flag_str || flag_str_str)
//     warn_sink_i16 = (int16)(warn_sink_i16 + 1);

// element_handle();
```

这段代码在 `feature/adaptive-find-lines` 分支上是启用状态，并且元素函数多接收一个
编码器里程参数 `encoder_data_1`（用于环岛阶段推进与停车线解锁）。里程由
`g_encoder_delta_accum` 在中断中累加，`image_process()` 里以
`interrupt_global_disable()` / `interrupt_global_enable()` 临界区读取后清零，
避免与 PWM 中断争用。

### 5.2 元素与判据

| 元素 | 判定入口 | 主要判据 | 输出标志 | 后处理 |
|---|---|---|---|---|
| 环岛（右） | `cir1_judge()` | 右边界方向直方图与丢线特征满足入场条件，且当前不在十字 | `flag_r_cir` | `cir_r_handle()`，由 `cir_stage` 分阶段推进 |
| 环岛（左） | `cir2_judge()` | 同上，镜像 | `flag_l_cir` | `cir_l_handle()` |
| 十字 | `cross_judge()` | 左右边界的方向序列与转折统计 | `flag_cross` | `l_border_cross()` / `r_border_cross()` 后重算中线 |
| 停车线 / 斑马线 | `stop_line()` | 固定行扫描「白 + 3 黑」条纹模式的命中次数，且里程满足解锁条件 | `flag_stop` | 由控制层执行停车保持（`g_stop_active`） |
| 障碍 / 条状物 | `judge_bar()` | 边界跳变次数：`abs(border[i] - border[i+3])` 超阈值的计数 | `flag_bar` | 提示 / 降速策略 |
| 直道（粗判） | `straight_judge()` | 巡线长度 > 0.75 H，且第 13–20 行与相隔 50 行的中线差 ≤ 5 px | `flag_str` | 无需修线 |
| 直道（严格） | `straight_judge_2()` | 同上，长度 > 0.8 H，且中线差 ≤ 2 px | `flag_str_str` | 无需修线 |

### 5.3 为什么环岛与十字是状态机

环岛一旦进入，后续行为不能只靠单帧判据决定。工程用 `cir_stage` 配合里程与丢线计数
推进阶段，不同阶段施加不同的修线方式（例如强制平移一侧边界、固定另一侧边界）。

十字则面临一个反向问题：**刚出十字时容易误判**。因此有 `cross_delay_judge()`
（`main_cm4.c:2797`）做延时恢复——识别到十字后在若干帧内抑制重复触发。

元素判据在 `element_test()` 中的优先级顺序是：右环岛 → 左环岛 → 十字
（`main_cm4.c:3299-3329`），命中高优先级元素时会清掉其他标志，避免同时置位。

详见 [`docs/element-recognition.md`](docs/element-recognition.md)。

---

## 6. 控制链路

控制分为外环（方向）与内环（速度），两者在不同周期的中断中运行。

### 6.1 多行加权误差

`calculate_line_error()`（`main_cm4.c:2141`）不只看某一行的中心偏差，而是在
第 10–40 行之间**线性取 10 个采样行**，对每行的偏差做加权平均，权重自上而下为 10…1：

```c
row = row_start + (row_end - row_start) * i / (sample_n - 1);
err = center_line[row] - MT9V03X_W / 2;
sum += (sample_n - i) * err;
```

越靠近车头（图像下方）的行权重越小——这是有意的：近处偏差大但已无法挽回，
远处的偏差才是转向需要提前响应的信息。采样行范围可在菜单中修改并写入 Flash。

### 6.2 转向 PID：D 项取陀螺角速度

位置式 PID，`Kp = 15.1`、`Ki = 0.00`、`Kd = 1.9`（`main_cm4.c:399`）。

D 项的信息源**不是**误差的差分，而是 IMU 的 Z 轴角速度：

```c
g_steering_pid.output = Kp * error + Ki * integral + Kd * gyro_z_filt;
```

理由：视觉误差本身噪声较大，对它做差分会放大噪声，表现为方向抖动；而陀螺仪直接测量
车体旋转速率，是同一物理量的更干净来源。

角速度先经过三级处理（`main_cm4.c:1455-1471`）：

| 处理 | 参数 | 目的 |
|---|---|---|
| 一阶低通 | α = 0.10 | 抑制高频噪声 |
| 死区 | ±3 dps | 静止时不输出，避免积分漂移与舵机嗡鸣 |
| 限幅 | ±200 dps | 防止异常读数（如碰撞）造成舵机猛打 |

输出经限幅后叠加到舵机中位：

```c
target_duty = g_servo_center_duty - g_steering_pid.output;
target_duty = clamp(target_duty, g_servo_min_duty, g_servo_max_duty);
```

### 6.3 舵机标定与 duty→转角映射

舵机控制本质是「PWM 脉宽 → 转角」的映射。软件输出的 duty 与真实前轮转角之间，
隔着舵机个体差异、连杆安装位置与 PWM 频率，因此必须标定三个值：
`g_servo_min_duty` / `g_servo_center_duty` / `g_servo_max_duty`（默认 3713 / 4400 / 5000）。

标定结果不只影响舵机能否回中，还直接决定阿克曼差速的正确性——因为差速需要「转角」这个量：

```c
static float servo_duty_to_steer_rad(uint32 duty)   // main_cm4.c:71
```

该函数把 duty 按「中位到两端」分段线性映射为弧度，上限为 `ACKERMANN_MAX_STEER_DEG`（15°）。

### 6.4 阿克曼几何差速

转弯时内外侧车轮走过的圆弧半径不同，外轮必须更快，否则推头或打滑。工程用标准阿克曼几何
把转角换算成左右轮速度比例：

```
geom  = track / (2 * wheelbase)          // 160 / (2 × 210) ≈ 0.381
scale = DIFF_SIGN * geom * tan(steer_rad)
scale = clamp(scale, ±g_ackermann_scale_limit)
v_l   = base * (1 - scale * g_ackermann_scale_limit)
v_r   = base * (1 + scale * g_ackermann_scale_limit)
```

> 几何参数取自 `main` 分支：轴距 210 mm、轮距 160 mm、最大前轮转角 15°。
> `feature/adaptive-find-lines` 分支用的是另一组值（155 mm / 115 mm / 15°），见[第 9 节](#9-分支说明)。

最后对 `g_target_speed_l/r` 做 `[0, g_max_target_speed]` 限幅。

`g_ackermann_scale_limit` 在这段代码中同时承担限幅与增益两个角色，详见
[第 12 节](#12-已知问题与限制)。

### 6.5 直线加速策略

基准速度默认 350（编码器脉冲 / 10 ms），可在满足**两个条件**时加速 `+80`：

```c
if (gyro_abs < g_straight_gyro_thr_dps &&   // 20 dps
    img_err_abs < g_straight_img_err_thr)   // 10 px
```

即「车身几乎不转、且偏离中线很小」才加速。这里只做直道加速，**不做弯道减速**——
弯道的速度分配由阿克曼差速负责。

### 6.6 速度环：增量式 PID

内环在 10 ms 的 PIT 中断中执行（`cm4_isr.c:40`），左右轮各一个控制器，参数
`Kp = 16.8`、`Ki = 2.5`、`Kd = 0.0`。

采用**增量式**形式，输出直接叠加到电机输出量上：

```c
increment = Kp*(e - e1) + Ki*e + Kd*(e - 2*e1 + e2);
pid->output = increment;
...
g_motor_out_l += g_speed_pid_l.output;
```

实际速度由正交编码器读取，单位为「每 10 ms 的脉冲数」，与控制周期严格对应：

```c
g_actual_speed_l = -encoder_get_count(ENCODER_L);   // 左轮取反，使左右轮同号
encoder_clear_count(ENCODER_L);
```

电机输出上限为 `g_max_pwm_duty`（默认 30，量程 100），并做 `[0, 上限]` 限幅。

### 6.7 三重保护

速度环中断在执行前依次检查三个入口条件，任一命中就把 PWM 置 0 并清空 PID 状态：

| 条件 | 触发场景 |
|---|---|
| `g_menu_state` 不在运行/调试模式 | 用户正在菜单里改参数 |
| `g_run_enable == false` | 运行模式但尚未发车 |
| `g_track_lost == true` | 丢线（底部 5 行黑点占比 > 82%，`check_track_lost()`，`main_cm4.c:3423`） |
| `g_stop_active == true` | 停车线触发后的停车保持 |

清空 PID 状态（而不只是输出 0）是必要的：否则恢复运行时残留的积分与历史误差会导致突跳。

详见 [`docs/control-and-tuning.md`](docs/control-and-tuning.md)。

---

## 7. 参数持久化

实车调参是迭代过程。如果每次上电都回到默认值，就无法复现同一套效果，调参成本会非常高。
因此工程把 **26 个**可调参数写入 Flash，上电时先加载再运行。

| 项 | 值 |
|---|---|
| 魔数 | `0xABCD1240` |
| 页索引 | 0 |
| 参数个数 | 26 |
| 保存 | `params_save_to_flash()`（`main_cm4.c:3460`） |
| 加载 | `params_load_from_flash()`（`main_cm4.c:3503`） |

保存时把参数按固定顺序打包进 `flash_union_buffer`，第 0 项写魔数，然后整页写入。
加载时**先校验魔数**：

- 魔数正确 → 说明这一页确实是本工程写入的，按顺序解包覆盖默认值
- 魔数不正确 → Flash 尚未写过或被污染，**保留代码中的默认值**

加载后对个别参数做二次范围检查（例如 `g_ackermann_scale_limit` 被限制在 `[0, 3]`），
避免 Flash 中的脏数据导致异常行为。

参数清单见 [`docs/flash-params.md`](docs/flash-params.md)。

---

## 8. 调试菜单

主循环是 `g_menu_state` 状态机，共 9 个状态：

| 状态 | 用途 |
|---|---|
| `MENU_MAIN` | 主菜单 |
| `MENU_RUN_MAIN_PROGRAM` | 正式运行 |
| `MENU_SPEED_DEBUG` | 速度环 PID 调参 |
| `MENU_PARAM_DEBUG` | 通用参数调参 |
| `MENU_THRESHOLD_DEBUG` | 阈值与图像调试（显示二值图、边界、中线、巡线轨迹点） |
| `MENU_PREVIEW_VARS` | 变量总览 |
| `MENU_SERVO_DEBUG` | 舵机标定 |
| `MENU_IMU_DEBUG` | IMU 数据查看 |
| `MENU_TRACKING_PARAM_DEBUG` | 巡线参数（前瞻行、宽度模型等） |

阈值调试菜单是本工程最有用的调试入口：它把二值图、左右边界（绿/红）、中线和八邻域的
原始轨迹点（青色）叠加显示在一块 180 × 120 的屏幕上，调视觉参数时无需借助上位机。

推荐的调参顺序与因果链见 [`docs/control-and-tuning.md`](docs/control-and-tuning.md)。

---

## 9. 分支说明

本仓库有两条分支，来自同一工程在 2026-01-16 之后的两个走向。**它们的差别不只是代码风格，
而是功能启用状态不同**，请按需选择：

| 项 | `main` | `feature/adaptive-find-lines` |
|---|---|---|
| 分支起点 | 2026-01-24（领先 6 个提交） | 2026-01-16 |
| 自研代码行数（两个源文件） | 4,129 | 4,164 |
| **元素识别调用** | **已注释，不执行** | **已启用** |
| 元素函数签名 | 无里程参数 | 接收 `encoder_data_1` 里程 |
| **姿态处理** | 只用 Z 轴角速度（一阶低通 + 死区 + 限幅） | **Mahony 六轴四元数 + 陀螺零偏标定** |
| 姿态融合开关 | — | `ATTITUDE_USE_ACC_FUSION`（默认 1，陀螺 + 加速度计） |
| **转向环周期** | **7,000 µs（≈143 Hz）** | **10,000 µs（100 Hz，与速度环同）** |
| **误差采样方式** | 行区间 10–40 内线性取 **10 点**，权重 10…1（**远处权重更大**） | **4 个固定行**（55 / 60 / 65 / 75），权重 0.2 / 0.2 / 0.3 / 0.3（**近处权重更大**） |
| 转向 PID | Kp 15.1 / Ki 0.00 / Kd 1.9 | Kp 9.5 / Ki 0.00 / Kd 0.1 |
| 速度 PID | Kp 16.8 / Ki 2.5 / Kd 0.0 | Kp 6.0 / Ki 0.3 / Kd 0.0 |
| 阿克曼差速系数 | 变量 `g_ackermann_scale_limit` = 1.2，**先限幅再乘一次** | 宏 `ACKERMANN_SCALE_LIMIT` = 0.95，只做限幅 |
| 舵机 duty（min / max / 中位） | 3713 / 5000 / 4400 | 3833 / 4900 / 4400 |
| 阿克曼轴距 / 轮距 | 210 mm / 160 mm | 155 mm / 115 mm |
| 直线加速策略 | 有（+80，需角速度 < 20 dps 且误差 < 10 px） | 无 |
| 大津法更新间隔 | 每 6 帧 | 每 10 帧 |
| 丢线判据（黑点占比） | 82% | 80% |
| 帧率统计 | DWT 周期计数器 | 无 |
| 摄像头曝光上限 | 400 | 500 |
| 元素识别函数命名 | 已整理 | 保留 `*_ported` 后缀 |

两条分支的差异集中在 3 个文件：`project/user/main_cm4.c`、`project/user/cm4_isr.c`
与 `libraries/zf_device/zf_device_mt9v03x.h`。

> 本仓库的 `README.md` 与 `docs/` 在两条分支上是**同一份文件**，其中的具体数值
> **默认以 `main` 分支为准**。阅读 `feature/adaptive-find-lines` 时请对照本表。

### 9.1 两条线的技术取向

两条线代表转向控制中「姿态信息从哪里来」的两种做法：

- **`feature/adaptive-find-lines`**：用完整的六轴姿态解算（Mahony 互补滤波）得到姿态角，
  并在上电时静止采样 200 次（约 400 ms）标定陀螺零偏。元素识别与修线全部接入，
  元素函数接收编码器里程参数以推进环岛阶段、解锁停车线。
- **`main`**：回到只取 Z 轴角速度，配一阶低通（α = 0.10）、死区（±3 dps）与限幅（±200 dps），
  直接作为转向 PID 的 D 项。同时把元素识别暂时关闭，专注于八邻域巡线本身的调试。

后者计算量小得多，也避开了姿态解算中横滚/俯仰对偏航的耦合误差——对只需要单轴信息的
转向控制而言更直接。

> **若你要一个功能完整的起点，请从 `feature/adaptive-find-lines` 开始。**

---

## 10. 构建与运行

### 10.1 环境

| 项 | 版本 |
|---|---|
| IDE | IAR Embedded Workbench for ARM **9.40.1** |
| 工作区 | `project/iar/cyt2bl3.eww` |
| 工程 | `project/iar/project_config/cyt2bl3.ewp` |
| 链接脚本 | `project/iar/icf/linker_directives_tviibe4m.icf` |

### 10.2 步骤

1. 用 IAR 打开 `project/iar/cyt2bl3.eww`
2. 首次打开时 IAR 会重建 `settings/` 目录（本仓库不含该目录，见 `NOTICE.md` §3.1）
3. 选择 `Debug_m4` 配置并编译
4. 连接调试器烧录

### 10.3 编码

`libraries/` 下逐飞库的源文件为 **GBK**，`project/user/` 下自研代码为 **UTF-8**。
若你的编辑器默认按 UTF-8 打开并自动转换编码，会导致中文注释被破坏——本工程的两个
文件就曾因此损坏（见 `NOTICE.md` §1.4）。**请关闭编辑器的自动编码推断，或为
`libraries/` 单独指定 GBK。**

### 10.4 未验证项

> **本仓库的代码未在命令行环境验证编译。** IAR 是仅图形界面的商业工具链，构建只能在
> IDE 内进行，本次整理过程未执行编译，也未进行实车验证。
>
> 因此：仓库中不包含任何编译产物、构建日志或赛道测试数据；README 与文档中所有
> 参数值均直接取自源码，而不是取自实测报告。详见 [`docs/PUBLICATION.md`](docs/PUBLICATION.md)。

---

## 11. 关键参数速查

| 类别 | 参数 | 默认值 | 位置 |
|---|---|---|---|
| 时钟 | 系统时钟 | 160 MHz | `main_cm4.c:367` |
| 控制周期 | 速度环 | 10,000 µs | `main_cm4.c:41, 412` |
| | 转向环 | 7,000 µs | `main_cm4.c:413` |
| 转向 PID | Kp / Ki / Kd | 15.1 / 0.00 / 1.9 | `main_cm4.c:399` |
| 速度 PID | Kp / Ki / Kd | 16.8 / 2.5 / 0.0 | `main_cm4.c:400-401` |
| 陀螺处理 | 低通 α / 死区 / 限幅 | 0.10 / ±3 dps / ±200 dps | `main_cm4.c:1456-1458` |
| 速度 | 基准速度 / 上限 | 350 / 1000 脉冲每 10 ms | `main_cm4.c:95, 119` |
| 直线加速 | 增量 / 角速度阈值 / 误差阈值 | +80 / 20 dps / 10 px | `main_cm4.c:122-124` |
| 阿克曼 | 轴距 / 轮距 / 最大转角 | 210 mm / 160 mm / 15° | `main_cm4.c:48, 52, 56` |
| 舵机 | 最小 / 中位 / 最大 duty | 3713 / 4400 / 5000 | `main_cm4.c:20-22` |
| 电机 | PWM 频率 / 最大占空比 | 17 kHz / 30% | `main_cm4.c:1420`, `130` |
| 图像误差 | 采样行范围 / 采样点数 | 10–40 / 10 | `main_cm4.c:127-128, 2147` |
| 赛道宽度 | 顶部 / 底部 | 60 / 180 px | `main_cm4.c:103-104` |
| 八邻域 | 起点行 / 步数上限 | 90 / 1500 | `main_cm4.c:2002, 2006` |
| 大津法 | 降采样步长 / 更新间隔 | 2 / 每 6 帧 | `main_cm4.c:137, 147` |
| 二值滤波 | 行范围 / 间隔 | 1–40 / 每 3 帧 | `main_cm4.c:167-171, 162` |
| 丢线判据 | 检查行数 / 黑点占比 | 5 行 / 82% | `main_cm4.c:3425-3426` |
| Flash | 魔数 / 参数个数 | `0xABCD1240` / 26 | `main_cm4.c:108-110` |

---

## 12. 已知问题与限制

本节如实列出整理过程中发现的问题，**未在本次开源整理中修改代码**。

> 最需要注意的一条是功能启用状态：**`main` 分支上元素识别未启用**（调用被注释）。
> 该事实已在[第 5 节](#5-元素识别)与[第 9 节](#9-分支说明)说明，此处不重复。

### 12.1 `g_ackermann_scale_limit` 同时充当限幅与增益

`steering_control()`（`main_cm4.c:2197-2219`）中：

```c
scale = clampf(scale, -g_ackermann_scale_limit, g_ackermann_scale_limit);
...
v_l = base * (1.0f - scale * g_ackermann_scale_limit);
v_r = base * (1.0f + scale * g_ackermann_scale_limit);
```

同一个常量先作为限幅边界、再作为增益系数，而它的注释与命名（"差速比例限幅（保护）"）
只反映了前者。在 `main` 分支的默认参数下几何量 `geom * tan(15°) ≈ 0.102`，所以该系数实际
等效为一个约 1.2 倍的差速增益，行为正常；但若把它调大，`scale * limit` 会以平方量级放大，
内侧轮的目标速度可能被压到 0（随后被 `[0, g_max_target_speed]` 限幅截断）。

> **该问题只存在于 `main` 分支。** `feature/adaptive-find-lines` 分支的写法是
> `v = base * (1 ± scale)`，`ACKERMANN_SCALE_LIMIT`（0.95）只用于限幅，不存在双重缩放。

### 12.2 无实车验证记录

仓库中不含赛道测试数据、成绩或视频。所有参数值取自源码，不代表实测最优值。

### 12.3 未在本机验证编译

见 [10.4](#104-未验证项)。

### 12.4 芯片厂商 SDK 的许可

`libraries/sdk/` 的许可未授予公开再分发权利。详见 [`NOTICE.md`](NOTICE.md) §2。

### 12.5 中断服务函数中的占位实现

`project/user/cm4_isr.c` 共 46 个函数定义，其中 22 个是空函数体——它们是逐飞库
预留的中断入口占位。真正实现功能的只有 24 个。

### 12.6 两个未真正参与控制的持久化参数

Flash 参数表中有两项在控制回路里没有被读取（仅被菜单修改、被 Flash 读写）：

| 参数 | 默认值 | 实际情况 |
|---|---|---|
| `g_speed_pid_integral_threshold` | 75 | 在菜单中可调并写入 Flash，但速度环的 `speed_pid_calculate()` 从未引用它。看起来是给积分分离准备的阈值，尚未接入 |
| `g_min_start_duty` | 15.0 | 只在一处被用到：调参菜单里作为 `g_max_pwm_duty` 的下限保护。**没有**作为电机启动时的最小占空比使用，与其命名和注释（"电机最小启动占空比"）不符 |

这两项不影响功能正确性，但会让阅读者误解参数含义，建议在后续版本中接入或移除。

### 12.7 死代码

| 位置 | 说明 |
|---|---|
| `fill_center_lines()`（`main_cm4.c:1842`，约 53 行） | 定义后**从未被调用**。中线实际是在 `image_process()` 末尾内联生成的（`main_cm4.c:2071-2094`）。两条分支都存在这个函数 |
| `static void width_change(void);`（`main_cm4.c:323`） | 只有前置声明，既无定义也无调用。`feature/adaptive-find-lines` 分支中不存在 |

这两处不影响行为，但会增加阅读成本。

---

## 13. 许可

本工程以 **GNU General Public License v3.0 or later** 发布，全文见 [`LICENSE`](LICENSE)。

之所以是 GPL-3.0：自研代码静态链接了逐飞科技 CYT2BL3 开源库，而该库以 GPL-3.0-or-later 发布。

第三方组件的版权与改动情况见 [`NOTICE.md`](NOTICE.md)。

---

## 文档索引

| 文档 | 内容 |
|---|---|
| [`docs/architecture.md`](docs/architecture.md) | 分层依赖、启动时序、前后台分工、中断资源 |
| [`docs/image-pipeline.md`](docs/image-pipeline.md) | 大津法、二值化、八邻域追踪、中线生成、兜底策略 |
| [`docs/element-recognition.md`](docs/element-recognition.md) | 元素判据、状态机阶段、修线策略 |
| [`docs/control-and-tuning.md`](docs/control-and-tuning.md) | 控制链路、舵机标定、调参顺序与因果链 |
| [`docs/flash-params.md`](docs/flash-params.md) | Flash 参数清单与持久化机制 |
| [`docs/pinout.md`](docs/pinout.md) | 引脚与片上资源分配 |
| [`docs/build.md`](docs/build.md) | IAR 构建与烧录、编码注意事项 |
| [`docs/PUBLICATION.md`](docs/PUBLICATION.md) | 整理范围、排除项、验证方法、未验证声明 |
