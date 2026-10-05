# 控制链路与调参

本文说明从图像误差到电机与舵机输出的完整控制链路，以及实车调参的推荐顺序。
对应 `project/user/main_cm4.c` 的控制函数与 `project/user/cm4_isr.c` 的两个中断。

> **分支差异提示**：本文按 **`main` 分支**的代码描述，其中下列数值在
> `feature/adaptive-find-lines` 分支上不同——请对照
> [README 第 9 节](../README.md#9-分支说明)的完整对照表：
>
> | 项 | `main` | `feature/adaptive-find-lines` |
> |---|---|---|
> | 转向环周期 | 7,000 µs | 10,000 µs（与速度环同） |
> | 误差采样 | 行 10–40 取 10 点，权重 10…1（远处权重大） | 4 个固定行 55/60/65/75，权重 0.2/0.2/0.3/0.3（近处权重大） |
> | 转向 PID | 15.1 / 0.00 / 1.9 | 9.5 / 0.00 / 0.1 |
> | 速度 PID | 16.8 / 2.5 / 0.0 | 6.0 / 0.3 / 0.0 |
> | 阿克曼差速系数 | 变量 1.2，先限幅再乘一次 | 宏 0.95，只做限幅 |
> | 轴距 / 轮距 | 210 / 160 mm | 155 / 115 mm |
> | 直线加速 | 有 | 无 |
>
> 其中**误差采样方式的差异最重要**：`main` 让远处（图像上方，第 10–40 行）权重更大，
> 属于「提前响应」；`feature/adaptive-find-lines` 让近处（第 55–75 行）权重更大，
> 属于「贴近当前姿态」。两者对转向手感的影响方向相反。

---

## 1. 控制结构总览

```
                    ┌──────────────── 外环（转向） ────────────────┐
 中线 ─▶ 多行加权误差 g_final_error                                │
                    │                                              │
   IMU ─▶ Z 轴角速度 ─▶ 低通 ▶ 死区 ▶ 限幅 ─▶ D 项                 │
                    │                                              │
                    ▼                                              │
              转向 PID（位置式，15.1 / 0.00 / 1.9）                  │
                    │                                              │
                    ├──▶ 舵机 duty = clamp(中位 − PID输出)  ──▶ 舵机 PWM
                    │                                              │
                    └──▶ duty ─▶ 转角(rad) ─▶ 阿克曼几何 ─▶ 差速比例 │
                                              │                    │
                                              ▼                    │
                                   左右轮目标速度 g_target_speed_l/r │
                                              │                    │
                    ┌─────────────────────────┼────────────────────┘
                    │        内环（速度）      ▼
  编码器 ─▶ 实际速度 ─▶ 增量式 PID（16.8 / 2.5 / 0.0）─▶ 电机 PWM
                    └────────────────────────────────────────────────┘
```

两个环在不同周期的中断里运行：**速度环 10 ms（100 Hz）**、**转向环 7 ms（≈143 Hz）**。
转向周期更短，是因为它需要跟上视觉更新的响应要求；而速度环的周期直接定义了速度的单位
（「每 10 ms 的编码器脉冲数」），改动它会同时改变所有速度参数的含义。

---

## 2. 外环：转向控制

### 2.1 误差计算：多行加权

`calculate_line_error()`（`main_cm4.c:2141`）不是只看某一行的中心偏差，而是在指定行区间内
均匀取 **10 个采样行**，做加权平均：

```c
int row_start = g_img_error_row_start;   // 默认 10
int row_end   = g_img_error_row_end;     // 默认 40
const int sample_n = 10;

for (int i = 0; i < sample_n; i++) {
    int row = row_start + (row_end - row_start) * i / (sample_n - 1);
    float err = center_line[row] - (MT9V03X_W / 2);
    float w   = (float)(sample_n - i);          // 10, 9, ..., 1
    sum  += w * err;
    wsum += w;
}
g_final_error = sum / wsum;
```

几个设计点：

- **线性插值取行**：`row_start` 与 `row_end` 可在菜单中改并写入 Flash，
  采样行随之自动重算（默认区间下为第 10、13、16、20、23、26、30、33、36、40 行）。
- **权重自上而下递减**：图像上方的行（远处）权重最大。近处偏差大但已无法挽回，
  远处的偏差才是转向需要提前响应的信息。
- **边界保护**：`row_end == row_start` 时会强制拉开一行，避免除零；
  行号都经 `clamp_row_u8()` 限幅。

### 2.2 转向 PID

位置式（`steering_pid_calculate()`，`main_cm4.c:1439`）：

```c
output = Kp * error + Ki * integral + Kd * gyro_z_filt;
```

| 参数 | 默认值 |
|---|---:|
| `Kp` | 15.1 |
| `Ki` | 0.00 |
| `Kd` | 1.9 |

积分项带上下限（`±g_speed_pid_integral_max`，默认 1000）。默认 `Ki = 0`，
因此积分通道实际不起作用——这一点在调参时需要注意：菜单里改 `Ki` 会生效，
但默认值下没有任何积分作用。

### 2.3 D 项的信息源：陀螺角速度

D 项**不是**误差的差分，而是 IMU 的 Z 轴角速度直接乘 `Kd`：

```c
float d_term = g_steering_pid.Kd * gyro_z_filt;
```

**为什么这么做**：PID 的 D 项在物理上要的是「被控量的变化趋势」。视觉误差做差分会把
噪声放大（相邻两帧的中心线抖动几个像素，除以很小的 `dt` 就变成很大的导数），
表现为方向抖动。而陀螺仪直接测量车体的旋转速率，是同一物理量的更干净来源。

角速度在使用前经过三级处理（`main_cm4.c:1455-1471`）：

| 处理 | 参数 | 作用 |
|---|---|---|
| 一阶低通 | `gyro_lpf_alpha = 0.10` | 抑制高频噪声。`gyro_z_filt += α * (raw − gyro_z_filt)` |
| 死区 | `gyro_deadband_dps = 3.0` | 幅值小于 3 dps 时直接归零，避免静止时的微小读数经 `Kd` 放大后让舵机持续嗡鸣 |
| 限幅 | `gyro_clamp_dps = 200.0` | 防止异常读数（碰撞、丢帧）造成舵机猛打角 |

处理顺序是「先低通、再判死区、最后限幅」，这一点在阅读代码时容易看错：
死区判断的是**滤波后**的值，而不是原始值。

### 2.4 舵机输出与标定

```c
int32_t target_duty = (int32_t)g_servo_center_duty - (int32_t)g_steering_pid.output;
if (target_duty < g_servo_min_duty) target_duty = g_servo_min_duty;
if (target_duty > g_servo_max_duty) target_duty = g_servo_max_duty;
pwm_set_duty(SERVO_PWM_PIN, g_servo_duty);
```

注意符号：**PID 输出是从中位 duty 中减去的**。因此正的误差（中线在右侧）会导致 duty 变小，
具体转向方向取决于舵机的安装方向。

| 标定值 | 默认 | 含义 |
|---|---:|---|
| `g_servo_min_duty` | 3713 | 左极限对应的 duty |
| `g_servo_center_duty` | 4400 | 回中对应的 duty |
| `g_servo_max_duty` | 5000 | 右极限对应的 duty |

**标定为什么重要**：duty 只是软件输出量，它对应的真实转角取决于舵机个体差异、连杆安装位置
以及 PWM 频率（本工程 300 Hz）。标定不准的后果有两层：

1. 舵机无法可靠回中，直道会持续跑偏；
2. **阿克曼差速算错**——因为差速需要「转角」这个物理量，而转角是由 duty 映射出来的
   （见下节）。映射不准，左右轮的目标速度比例就会错，表现为推头或甩尾。

标定在 `MENU_SERVO_DEBUG` 菜单中完成。

### 2.5 duty → 转角映射

`servo_duty_to_steer_rad()`（`main_cm4.c:71`）把 duty 按「中位到两端」分段线性映射为弧度：

```c
const float max_angle_rad = ACKERMANN_MAX_STEER_DEG * (π / 180);
if (duty >= center) {
    t = clamp((duty - center) / (max_duty - center), 0, 1);
    return  t * max_angle_rad;
} else {
    t = clamp((center - duty) / (center - min_duty), 0, 1);
    return -t * max_angle_rad;
}
```

分母有 `1.0f` 的下限保护，避免标定值相等时除零。

### 2.6 阿克曼几何差速

`main_cm4.c:2196-2219`：

```c
float steer_rad = servo_duty_to_steer_rad(g_servo_duty);
float tan_delta = tanf(steer_rad);
float geom  = ACKERMANN_TRACK_MM / (2.0f * ACKERMANN_WHEELBASE_MM);
float scale = ACKERMANN_DIFF_SIGN * geom * tan_delta;
scale = clampf(scale, -g_ackermann_scale_limit, g_ackermann_scale_limit);

float v_l = base * (1.0f - scale * g_ackermann_scale_limit);
float v_r = base * (1.0f + scale * g_ackermann_scale_limit);
```

几何参数（`main` 分支）：

| 参数 | 值 | 含义 |
|---|---:|---|
| `ACKERMANN_WHEELBASE_MM` | 210.0 | 轴距（mm） |
| `ACKERMANN_TRACK_MM` | 160.0 | 后轮轮距，即左右驱动轮中心距（mm） |
| `ACKERMANN_MAX_STEER_DEG` | 15.0 | 舵机打满时的最大前轮转角 |
| `ACKERMANN_DIFF_SIGN` | −1.0 | 差速方向符号，用于匹配舵机安装方向 |

**物理依据**：车辆转弯时，内外侧车轮走过的圆弧半径不同。前轮转角为 δ 时，
内外轮速度比满足

```
v_outer / v_inner = (R + track/2) / (R − track/2),   R = wheelbase / tan(δ)
```

对小的 `track/(2R)` 展开即得一阶近似 `v = base * (1 ± track/(2·wheelbase) · tan δ)`，
正是代码中的形式。

> **注意**：`g_ackermann_scale_limit` 在这段代码中同时充当限幅边界与增益系数，
> 详见 README 的[已知问题](../README.md#121-g_ackermann_scale_limit-同时充当限幅与增益)。

### 2.7 基准速度与直线加速

```c
float base = (float)g_base_speed;               // 默认 350
float gyro_abs  = fabsf(imu660rb_gyro_transition(imu660rb_gyro_z));
float img_err_abs = fabsf(g_final_error);
if (gyro_abs  < g_straight_gyro_thr_dps &&      // 默认 20 dps
    img_err_abs < g_straight_img_err_thr) {     // 默认 10 px
    base += g_straight_speed_add;               // 默认 +80
}
```

即**只有「车身几乎不转」且「偏离中线很小」时才加速**。两个条件缺一不可：

- 只看图像误差不够——误差小可能是车正在平稳地走直线，也可能是控制环刚好在过零点的瞬间；
- 加上陀螺判据后，可以确认车体确实没有转动趋势。

这是「只做直道加速、不做弯道减速」的策略：弯道的速度分配交给阿克曼差速，
不需要额外的减速逻辑。

### 2.8 目标速度限幅

```c
if (g_target_speed_l < 0) g_target_speed_l = 0;
else if (g_target_speed_l > g_max_target_speed) g_target_speed_l = g_max_target_speed;
```

`g_max_target_speed` 默认 1000（脉冲 / 10 ms）。下界钳到 0 意味着目标速度不会为负——
工程不做倒车。

---

## 3. 内环：速度控制

`pit0_ch0_isr()`（`cm4_isr.c:40`），周期 10 ms。

### 3.1 测速

```c
g_actual_speed_l = -encoder_get_count(ENCODER_L);   // 左轮取反
encoder_clear_count(ENCODER_L);
g_actual_speed_r = encoder_get_count(ENCODER_R);
encoder_clear_count(ENCODER_R);
```

- **单位是「每 10 ms 的脉冲数」**，与中断周期严格绑定。
- **左轮取反**：两个电机的安装方向相反，取反后「正数 = 前进」对左右轮一致。
- **读完立即清零**：这样每次读取得到的都是本周期内的增量。

### 3.2 增量式 PID

`speed_pid_calculate()`（`main_cm4.c:1485`）：

```c
pid->error = target_speed - actual_speed;
float increment = pid->Kp * (pid->error - pid->last_error) +
                  pid->Ki * pid->error +
                  pid->Kd * (pid->error - 2*pid->last_error + pid->pre_last_error);
pid->output = increment;
pid->pre_last_error = pid->last_error;
pid->last_error = pid->error;
```

**为什么用增量式**：输出是「本次应该增加多少 PWM」，累加到电机输出量上：

```c
g_motor_out_l += g_speed_pid_l.output;
```

增量式天然带积分作用（累加即积分），且**不需要抗积分饱和逻辑**——
当输出被限幅时，下一周期的增量会自动减小。位置式则需要显式处理积分饱和。

参数：`Kp = 16.8`、`Ki = 2.5`、`Kd = 0.0`（左右轮相同）。

### 3.3 输出与限幅

```c
float max_pwm_val = g_max_pwm_duty * 100.0f;    // 30 * 100 = 3000

if (g_motor_out_l > max_pwm_val) g_motor_out_l = max_pwm_val;
else if (g_motor_out_l < 0)      g_motor_out_l = 0;

g_motor_out_l_duty = g_motor_out_l / 100.0f;    // 供显示用的百分比
pwm_set_duty(MOTOR_L_PWM_PIN, (uint32)g_motor_out_l);
```

需要注意单位换算：`g_motor_out_*` 的单位是**千分之一**（量程 0–10000 对应 0–100%），
而 `g_max_pwm_duty` 的单位是**百分比**，所以乘 100 后再比较。
`g_motor_out_l_duty` 是给屏幕显示用的百分比值，不参与 PWM 输出。

### 3.4 三重保护

中断在执行控制逻辑前依次检查以下条件，任一命中就把 PWM 置 0、清空 PID 全套状态、
清零编码器计数并返回：

| 条件 | 触发场景 | `cm4_isr.c` |
|---|---|---|
| `g_menu_state` 不在运行/速度调试模式 | 用户正在菜单里改参数 | L44-65 |
| `g_track_lost == true` | 丢线 | L66-87 |
| `g_stop_active == true` | 停车线触发后的停车保持 | L89-116 |

丢线判据 `check_track_lost()`（`main_cm4.c:3423`）：检查图像底部 5 行，
黑点占比超过 **82%** 判定为丢线。此时不仅清零输出，`run_main_program_task()`
还会退出运行模式并回到主菜单（`main_cm4.c:654-667`）。

清空的是 `output`、`error`、`last_error`、`pre_last_error`、`integral` 全套状态。
**只把输出置 0 是不够的**——恢复运行时残留的历史误差会让增量式 PID 产生突跳。

### 3.5 转向环的门控

`pit0_ch1_isr()`（`cm4_isr.c:163`）：

```c
if (g_menu_state == MENU_RUN_MAIN_PROGRAM) {
    if (!g_run_enable) return;
    if (!g_track_lost) steering_control();
    return;
}
if (g_menu_state == MENU_THRESHOLD_DEBUG) {
    steering_control_servo_only();   // 只动舵机，不动电机
    return;
}
```

阈值调试模式下用 `steering_control_servo_only()`（`main_cm4.c:3400`）：
它跑同样的转向 PID 与舵机输出，但**不计算阿克曼差速、不改目标速度**，
因此可以在车静止时观察舵机是否正确跟随中线。

---

## 4. 调参顺序

顺序不是随意的，每一步都依赖前一步的输出正确。

### 4.1 推荐顺序与因果链

| 步骤 | 菜单 | 为什么必须在这一步 |
|---|---|---|
| 1 | `MENU_THRESHOLD_DEBUG` | 阈值不对 → 二值图不对 → 边界与中线全错 → **后面所有 PID 怎么调都救不回来** |
| 2 | `MENU_SERVO_DEBUG` | 标定 min / center / max。映射不对 → 方向控制「拧麻花」，且阿克曼差速的比例也是错的 |
| 3 | `MENU_IMU_DEBUG` | 确认静止时角速度接近 0、转动时符号与幅值合理。D 项直接用它，读数不对会让转向剧烈抖动 |
| 4 | 转向 PID（`MENU_PARAM_DEBUG`） | 先把车能稳定压在中线上 |
| 5 | 速度 PID（`MENU_SPEED_DEBUG`） | 在「能压住线」的基础上提速。转向不稳时提速只会放大摆动 |
| 6 | 巡线参数（`MENU_TRACKING_PARAM_DEBUG`） | 前瞻行、权重范围、赛道宽度模型 |

### 4.2 每一步的判据

**1. 阈值与图像**
- 二值图中赛道应为白色连片区域，没有大面积空洞或噪点
- 绿色（左边界）与红色（右边界）应贴合赛道两侧
- 蓝色中线应位于赛道中央；青色轨迹点是八邻域的原始走向，应连续无跳变

**2. 舵机标定**
- 中位: 舵机回正，前轮朝正前方
- 最小 / 最大: 分别对应左右机械极限，且左右对称
- 标定完成后 duty 与真实转角应大致线性

**3. IMU**
- 静止时 Z 轴角速度读数应在 ±3 dps 内（落在死区内）
- 手动左转 / 右转时符号正确

**4. 转向 PID**
- 先把 `Kd` 设为 0，调 `Kp` 直到车能回中线但略有摆动
- 再加 `Kd` 抑制摆动。`Kd` 过大时表现为高频抖动（陀螺噪声被放大）
- `Ki` 默认 0，一般不需要动

**5. 速度 PID**
- 先给一个较低的 `g_base_speed`，调 `Kp` 直到实际速度能跟上目标
- `Ki` 用于消除稳态误差，过大引起低频振荡
- 稳定后再逐步提高 `g_base_speed` 与 `g_max_pwm_duty`

**6. 巡线参数**
- `g_img_error_row_start` / `_end` 决定前瞻距离：区间越靠上（行号越小），转向越提前
- `g_track_width_top` / `_bottom` 决定单侧丢线时的补线宽度

### 4.3 保存

在 `MENU_PARAM_DEBUG` 中触发保存，26 个参数写入 Flash（见 [Flash 参数](flash-params.md)）。
**下次上电自动加载**，因此调好一套后要保存，否则断电即丢失。

---

## 5. 相关文档

- [图像处理链路](image-pipeline.md)
- [元素识别](element-recognition.md)
- [Flash 参数](flash-params.md)
- [系统架构](architecture.md)
