# Flash 参数持久化

本文说明工程如何把可调参数保存到片上 Flash、上电如何加载，以及完整的 26 项参数清单。

> **分支差异提示（重要）**：本文的参数表以 **`main` 分支**为准。
> `feature/adaptive-find-lines` 分支的参数表**结构完全不同**：
>
> | 项 | `main` | `feature/adaptive-find-lines` |
> |---|---|---|
> | 魔数 | `0xABCD1240` | `0xABCD1239` |
> | 参数个数 | 26 | **29** |
> | 槽位 13–20 | 舵机 duty、目标速度上限、积分限幅、赛道宽度 | **误差采样行 1–4 与权重 1–4** |
> | 槽位 21–28 | 直线加速三项、误差采样行区间 | 舵机 duty、目标速度上限、积分限幅、赛道宽度 |
> | 槽位 11 | `g_ackermann_scale_limit` | `g_speed_steer_scale` |
>
> 两分支使用**不同的魔数**，因此旧数据会被正确拒绝而不是被错误解释——这是恰当的做法。
> 但跨分支烧录前仍建议先擦除 Flash 页。完整对照见
> [README 第 9 节](../README.md#9-分支说明)。

---

## 1. 为什么需要它

实车调参是一个迭代过程：改一个参数、跑一圈、看表现、再改。如果每次上电都回到代码里的
默认值，就无法复现刚才调好的那套效果，调参成本会高到不可接受。

因此工程把 **26 个**运行期可调参数写入 Flash，上电时先加载、再进入菜单。

| 项 | 值 |
|---|---|
| 魔数 | `PARAMS_FLASH_MAGIC = 0xABCD1240` |
| 页索引 | `PARAMS_FLASH_PAGE_INDEX = 0` |
| 参数个数 | `PARAMS_LEN = 26` |
| 保存 | `params_save_to_flash()`（`main_cm4.c:3460`） |
| 加载 | `params_load_from_flash()`（`main_cm4.c:3503`） |

---

## 2. 写入流程

```c
void params_save_to_flash(void)
{
    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = PARAMS_FLASH_MAGIC;
    flash_union_buffer[1].float_type  = g_steering_pid.Kp;
    /* ... 按固定顺序填充 1..25 ... */
    flash_write_page_from_buffer(0, PARAMS_FLASH_PAGE_INDEX, PARAMS_LEN);
    ips200_show_string(0, 180, "Parameters Saved!");
    system_delay_ms(500);
}
```

1. **清空缓冲区** — 避免上一次操作的残留数据被写入
2. **第 0 项写魔数** — 这是整套机制的完整性标记
3. **按固定顺序填充** — 顺序即协议，读写两侧必须严格对应
4. **整页写入** — `flash_write_page_from_buffer(0, 页号, 26)`

参数以 `flash_union_buffer[]` 的联合体形式存放，同一槽位可按 `uint32_type`、
`int32_type`、`float_type` 解释。这解决了「Flash 只能按 32 位字写入」与
「参数有 float 也有 int」之间的矛盾。

> **兼容性提醒**：参数顺序一旦变更（插入、删除、重排），旧 Flash 中的数据会被错误解释。
> 工程用魔数只能检测「这一页是不是本工程写的」，**检测不出参数表版本**。
> 修改参数表后应先擦除 Flash 或更换魔数。

---

## 3. 加载流程与魔数校验

```c
void params_load_from_flash(void)
{
    flash_read_page_to_buffer(0, PARAMS_FLASH_PAGE_INDEX, PARAMS_LEN);
    if (flash_union_buffer[0].uint32_type != PARAMS_FLASH_MAGIC) {
        return;                                  // 直接返回，保留代码中的默认值
    }
    g_steering_pid.Kp = flash_union_buffer[1].float_type;
    /* ... */
}
```

**魔数不正确时直接返回**，不做任何覆盖。这覆盖了三种情况：

| 情况 | 结果 |
|---|---|
| Flash 从未写过（新芯片 / 已擦除） | 读出全 `0xFF`，魔数不匹配 → 用默认值 |
| 数据被污染或写入中断 | 魔数大概率不匹配 → 用默认值 |
| 参数表版本变更 | **魔数仍匹配**，会用新顺序解释旧数据（见上节提醒） |

### 3.1 加载后的二次范围检查

个别参数在加载后有额外的范围钳制，防止 Flash 中的脏数据导致异常行为：

```c
if (g_ackermann_scale_limit < 0.0f) g_ackermann_scale_limit = 0.0f;
if (g_ackermann_scale_limit > 3.0f) g_ackermann_scale_limit = 3.0f;
```

这与菜单中的可调范围（0–3）一致，两处共同保证该参数不会超出安全区间。

---

## 4. 完整参数表

槽位 0 为魔数，1–25 为实际参数，共 26 项。

| 槽位 | 类型 | 变量 | 默认值 | 含义 |
|---:|---|---|---:|---|
| 0 | `uint32` | `PARAMS_FLASH_MAGIC` | `0xABCD1240` | 完整性标记 |
| 1 | `float` | `g_steering_pid.Kp` | 15.1 | 转向 PID 比例 |
| 2 | `float` | `g_steering_pid.Ki` | 0.00 | 转向 PID 积分 |
| 3 | `float` | `g_steering_pid.Kd` | 1.9 | 转向 PID 微分（作用于陀螺角速度） |
| 4 | `float` | `g_speed_pid_l.Kp` | 16.8 | 左轮速度 PID 比例 |
| 5 | `float` | `g_speed_pid_l.Ki` | 2.5 | 左轮速度 PID 积分 |
| 6 | `float` | `g_speed_pid_r.Kp` | 16.8 | 右轮速度 PID 比例 |
| 7 | `float` | `g_speed_pid_r.Ki` | 2.5 | 右轮速度 PID 积分 |
| 8 | `int32` | `g_base_speed` | 350 | 基准速度（脉冲 / 10 ms） |
| 9 | `float` | `g_max_pwm_duty` | 30.0 | 电机最大占空比（%） |
| 10 | `float` | `g_min_start_duty` | 15.0 | 见下方说明 |
| 11 | `float` | `g_ackermann_scale_limit` | 1.2 | 阿克曼差速系数（0–3） |
| 12 | `int32` | `g_dynamic_threshold` | 128 | 二值化阈值初值 |
| 13 | `int32` | `g_servo_min_duty` | 3713 | 舵机最小 duty |
| 14 | `int32` | `g_servo_max_duty` | 5000 | 舵机最大 duty |
| 15 | `int32` | `g_servo_center_duty` | 4400 | 舵机中位 duty |
| 16 | `int32` | `g_max_target_speed` | 1000 | 左右轮目标速度上限 |
| 17 | `int32` | `g_speed_pid_integral_threshold` | 75 | 见下方说明 |
| 18 | `float` | `g_speed_pid_integral_max` | 1000.0 | 转向 PID 积分限幅 |
| 19 | `int32` | `g_track_width_top` | 60 | 赛道宽度模型：顶部（px） |
| 20 | `int32` | `g_track_width_bottom` | 180 | 赛道宽度模型：底部（px） |
| 21 | `int32` | `g_straight_speed_add` | 80 | 直线加速增量（脉冲 / 10 ms） |
| 22 | `float` | `g_straight_gyro_thr_dps` | 20.0 | 直线加速允许的角速度上限（dps） |
| 23 | `float` | `g_straight_img_err_thr` | 10.0 | 直线加速允许的图像误差上限（px） |
| 24 | `int32` | `g_img_error_row_start` | 10 | 误差采样行区间起点 |
| 25 | `int32` | `g_img_error_row_end` | 40 | 误差采样行区间终点 |

### 4.1 两项未真正参与控制的参数

| 槽位 | 参数 | 实际情况 |
|---:|---|---|
| 17 | `g_speed_pid_integral_threshold` | 可在菜单中调整、会写入与读出 Flash，但速度环的 `speed_pid_calculate()` **从未引用它**。从命名推测是为积分分离准备的阈值，尚未接入 |
| 10 | `g_min_start_duty` | 只在调参菜单里作为 `g_max_pwm_duty` 的下限保护被引用一次。**没有**作为电机启动时的最小占空比使用，与其命名和注释（"电机最小启动占空比"）不符 |

两项都不影响功能正确性，但会让阅读者误解参数含义。

---

## 5. 保存时机

工程**不会**在每次改参数时自动写入 Flash——Flash 擦写次数有限，且写入期间会阻塞。
保存是显式动作：在 `MENU_PARAM_DEBUG` 菜单中选择保存项后调用 `params_save_to_flash()`，
屏幕上给出 `"Parameters Saved!"` 提示并延时 500 ms。

保存期间会阻塞主循环，但**不会影响控制**——PIT 中断独立运行。

---

## 6. 相关文档

- [控制链路与调参](control-and-tuning.md)
- [系统架构](architecture.md)
- [引脚与资源](pinout.md)
