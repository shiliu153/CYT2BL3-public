# 元素识别

本文说明赛道元素（环岛、十字、停车线、斑马线、障碍、直道）的识别方式与修线策略，
对应 `project/user/main_cm4.c` 中的 `element_*` / `*_judge` / `*_handle` 系列函数。

---

## 1. 设计：判定与处理分离

元素识别不是一个大函数一次识别所有元素，而是每帧按固定顺序执行的一组步骤：

```
stop_line(encoder)  →  cross_delay_judge()  →  element_test()
    →  straight_judge()  →  straight_judge_2()  →  judge_bar()
        →  element_handle()
```

并且严格区分两类职责：

| 职责 | 函数 | 做什么 | 不做什么 |
|---|---|---|---|
| **判定** | `element_test()`、`stop_line()`、`judge_bar()`、`straight_judge()`、`straight_judge_2()` | 只更新标志位（`flag_*`）与阶段变量（`cir_stage`） | 不改动边界 |
| **处理** | `element_handle()` | 按标志位修补 `left_border` / `right_border`，再重算中线 | 不做判定 |

这样划分有两个好处：

1. 判定逻辑可以独立于修线策略调试——两者出问题时表现不同，分开才好定位。
2. 阈值调试菜单可以分别显示「识别到了什么」（标志位）与「修线修成什么样」（边界与中线）。

> **启用状态（务必先读）**
>
> - **`main` 分支**：`image_process()` 内部的元素调用块（`main_cm4.c:2100-2112`）**被注释掉了**。
>   图像链路只做纯中线提取，元素判定与修线**不执行**。相关函数完整保留，恢复调用即可启用。
> - **`feature/adaptive-find-lines` 分支**：这些调用**已启用**，对应 `*_ported` 系列函数，
>   并且元素函数额外接收编码器里程参数 `encoder_data_1`。
>
> 本文描述的是这套逻辑本身的设计；两分支的实际代码差异见 README
> 的[分支说明](../README.md#9-分支说明)。

---

## 2. 判定的输入：巡线的副产物

元素判别几乎不重新扫描图像，而是复用八邻域追踪的副产物：

| 输入 | 来源 | 含义 |
|---|---|---|
| `left_border[]` / `right_border[]` | `extract_borders()` | 按行索引的边界位置（`-1` / `200` 表示丢失） |
| `dir_l[]` / `dir_r[]` | `search_eight_neighbor()` | 边界每一步的移动方向（0–7） |
| `l_dir_sum[]` / `r_dir_sum[]` | 同上 | 方向直方图 |
| `data_statics_l` / `data_statics_r` | `image_process()` | 左右边界轨迹的点数 |
| `length` | `lose_line()` | 巡线有效长度 |
| `encoder` 里程 | 编码器累计 | 用于解锁停车线与推进环岛阶段 |

方向直方图由 `sum_dir_l()` / `sum_dir_r()`（`main_cm4.c:2304` / `2272`）汇总，
曲率由 `process_curvity()`（`main_cm4.c:2725`）用三点计算：

```c
process_curvity(lx1, y20, lx2, y35, lx3, y50)   // 第 100、85、70 行三个边界点
l_line_qulv = 1000.0f * process_curvity(...)
```

即取左边界在三个纵向位置上的点，量化成「向左弯还是向右弯、弯多少」。

---

## 3. 元素与判据

| 元素 | 判定入口 | 主要判据 | 输出标志 | 后处理 |
|---|---|---|---|---|
| 环岛（右） | `cir1_judge()` | 右边界方向直方图与丢线特征满足入场条件，且当前不在十字 | `flag_r_cir = 1` | `cir_r_handle()`，由 `cir_stage` 分阶段推进 |
| 环岛（左） | `cir2_judge()` | 同上，镜像 | `flag_l_cir = 1` | `cir_l_handle()` |
| 十字 | `cross_judge()` | 左右边界的方向序列与转折统计 | `flag_cross` | `l_border_cross()` / `r_border_cross()` 后重算中线 |
| 停车线 / 斑马线 | `stop_line()` | 固定行扫描「白 + 3 黑」条纹模式，命中次数达阈值且里程满足解锁条件 | `flag_stop` | 由控制层执行停车保持（`g_stop_active`） |
| 障碍 / 条状物 | `judge_bar()` | 边界跳变次数：`abs(border[i] - border[i+3])` 超阈值的计数 | `flag_bar` | 提示 / 降速策略 |
| 直道（粗判） | `straight_judge()` | `length > 0.75 H` 且第 13–20 行与相隔 50 行的中线差 ≤ 5 px | `flag_str` | 无需修线 |
| 直道（严格） | `straight_judge_2()` | `length > 0.8 H` 且中线差 ≤ 2 px | `flag_str_str` | 无需修线 |

### 3.1 直道判据的实现

两条直道判据结构相同，只是阈值不同（`main_cm4.c:3332` / `3364`）：

```c
int length_straight_th = (MT9V03X_H * 3) / 4;      // 0.75 H = 90 行（严格版为 0.8 H = 96 行）
if (length_straight_th < 40) length_straight_th = 40;

if (length > length_straight_th && flag_l_cir != 1 && flag_r_cir != 1) {
    int i_begin  = scale_from_120(13);             // 换算到实际分辨率
    int i_end    = scale_from_120(20);
    int i_offset = scale_from_120(50);
    for (int i = i_begin; i < i_end; i++) {
        int i2 = i + i_offset;
        if (i2 >= MT9V03X_H) break;
        if (zf_abs_i(center_line_i16[i] - center_line_i16[i2]) > 5) {   // 严格版为 > 2
            flag_str = false;
            return 0;
        }
    }
    flag_str = true;
    return 1;
}
```

`scale_from_120()` / `scale_from_180()`（`main_cm4.c:278` / `283`）把按 120 / 180 基准写死的
行号、列号换算到实际分辨率，使算法在改变图像尺寸后仍然有效。

两个前置条件值得注意：

- **`flag_l_cir` / `flag_r_cir` 必须为 0**：环岛内部也存在长直边界，不排除会误判为直道。
- **点数 `length` 必须足够**：`length` 是巡线有效长度，短于此说明边界提取本身就不可靠。

---

## 4. 为什么环岛与十字是状态机

### 4.1 环岛：入场之后的持续行为

环岛一旦进入，后续行为不能只靠单帧判据决定。工程的做法是：

1. `element_test()` 中的 `cir1_judge()` / `cir2_judge()` 只负责**入场判定**，置 `flag_r_cir` / `flag_l_cir`
2. 入场后由 `cir_stage` 结合里程（`encoder_data_1`）与丢线统计推进阶段
3. 每个阶段施加不同的修线方式，例如：
   - 强制平移一侧边界到固定位置
   - 固定一侧边界不动、只让另一侧跟随
   - 在出环阶段逐步恢复真实边界

`cir_point()`（`main_cm4.c:2457`）记录环岛的参考点（`cir_x` / `cir_y`），
供各阶段的修线使用。`lose_location_test()`（`main_cm4.c:2426`）用于在指定行区间内
检查丢线情况，是阶段推进的主要依据之一。

### 4.2 十字：出十字时的误判抑制

十字面临一个与环岛相反的问题：**刚出十字时容易重复触发**。
解决方案是延时恢复子状态机：

| 变量 | 作用 |
|---|---|
| `cross_jud` | 延时状态：0 = 未触发延时，1 = 延时中 |
| `cross_jud_time` | 剩余延时帧数 |
| `cross_delay_judge()` | 每帧递减计数，归零后才允许再次判定 |

`cross_judge()`（`main_cm4.c:2670`）返回三态：

| 返回值 | 含义 | `element_test()` 中的处理 |
|---:|---|---|
| 1 | 进入十字 | `flag_cross = 1`，清 `flag_others2` |
| 2 | 处于十字中 | `flag_cross = 0`，清 `flag_others2` |
| 0 | 不在十字 | `flag_cross = 0`，`flag_others2 = 1` |

### 4.3 判定优先级

`element_test()`（`main_cm4.c:3299-3329`）中的优先级是
**右环岛 → 左环岛 → 十字**：

```c
if (cir1_judge() && flag_cross != 1) {
    flag_r_cir = 1; flag_cross = 0; flag_others2 = 0;
} else if (cir2_judge() && flag_cross != 1 && flag_r_cir != 1) {
    flag_l_cir = 1; flag_cross = 0; flag_others2 = 0;
} else {
    uint8 cj = cross_judge();
    ...
}
```

命中高优先级元素时会**清掉其他标志**，避免同时置位导致修线策略互相冲突。
环岛判定还带一个前置条件 `flag_cross != 1`——正在十字中就不判环岛。

---

## 5. 修线策略

`element_handle()`（`main_cm4.c:3223`）按标志位分派：

| 标志 | 修线方式 |
|---|---|
| `flag_r_cir` | `cir_r_handle()`（`main_cm4.c:2915`，约 160 行） |
| `flag_l_cir` | `cir_l_handle()`（`main_cm4.c:3077`，约 145 行） |
| `flag_cross` | `l_border_cross()` / `r_border_cross()` |
| `flag_stop` / `flag_bar` / `flag_str` | 通常不修线，只在控制层改变策略 |

修线的共同做法是：**先改动 `left_border` / `right_border`，再统一重算中线**。
这样中线永远与边界一致，不会出现「边界修了、中线没修」的不一致状态。

两个环岛处理函数各约 150 行，是工程中最长的两个元素函数——因为环岛的每个阶段
都要单独描述边界应该如何被改造。

---

## 6. 停车线与斑马线

`stop_line()`（`main_cm4.c:2741`）与其他元素不同：

- 它**不基于边界**，而是直接在灰度/二值图上按固定行扫描横向条纹
- 判据是「白 + 3 黑」模式的命中次数达到阈值
- 有一个**里程解锁**条件 `s_stop_arm`，避免在起跑区误触发
- 触发后**不在 `element_handle()` 中修线**，而是置 `flag_stop`，由控制层执行停车保持

停车保持由 `g_stop_active` 驱动（`main_cm4.c:133`、`cm4_isr.c:89-116`）:
速度环检测到该标志后把目标速度与电机输出清零、清空速度环积分，
使车停在原地而不是继续巡线。

---

## 7. 相关文档

- [图像处理链路](image-pipeline.md)
- [控制链路与调参](control-and-tuning.md)
- [系统架构](architecture.md)
