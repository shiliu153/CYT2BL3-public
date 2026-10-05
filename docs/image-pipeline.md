# 图像处理链路

本文说明从一帧灰度图到一条可用的中心线之间发生的全部处理，对应
`project/user/main_cm4.c` 中的 `image_process()` 及其调用链。

图像规格：**180 × 120** 灰度（`MT9V03X_W` / `MT9V03X_H`，见
`libraries/zf_device/zf_device_mt9v03x.h:74-75`）。传感器本身支持到 188 × 120，
本工程取 180 是为了同时启用硬件 SPI 屏幕的刷新。

> **分支差异提示**：本文按 **`main` 分支**的代码描述。`feature/adaptive-find-lines`
> 分支在图像链路本体上一致，不同之处是：大津法更新间隔为**每 10 帧**（`main` 为 6 帧）、
> 丢线判据的黑点占比阈值为 **80%**（`main` 为 82%），并且启用了元素识别与修线
> （`main` 中这些调用被注释）。完整对照见 [README 第 9 节](../README.md#9-分支说明)。

---

## 1. 流程总览

```
mt9v03x_image[120][180]   （灰度原图，由摄像头驱动 DMA 填充）
        │
        ▼  大津法动态阈值 g_dynamic_threshold
        │
        ▼  二值化  bin_image[120][180] ∈ {0, 255}
        │
        ▼  定向滤波（每 3 帧一次，仅第 1–40 行）
        │
        ▼  图像边界锁死 image_draw_rectan()
        │
        ▼  找起始点 get_start_points()（第 90 行）
        │
        ├──成功──▶ 八邻域追踪 search_eight_neighbor() ─▶ 边界提取 extract_borders()
        │
        └──失败──▶ 逐行扫描兜底 fallback_scanline()
        │
        ▼  中线生成 + 提前相遇补偿
        │
        ▼  写回 g_left_line / g_right_line / g_center_line
```

`image_process()` 的入口在 `main_cm4.c:1952`。

---

## 2. 动态阈值：大津法

### 2.1 原理

大津法（Otsu）把灰度直方图分成前景与背景两类，寻找使**类间方差最大**的阈值：

```
σ²_b(T) = ω₀(T) · ω₁(T) · [μ₀(T) − μ₁(T)]²
T* = argmax σ²_b(T)
```

其中 `ω` 是两类的像素占比，`μ` 是两类的灰度均值。直观理解：希望切出来的两堆
「相互差得尽量远，且各自内部尽量集中」。

实现见 `calculate_otsu_threshold()`（`main_cm4.c:1295`）。

### 2.2 为什么不用固定阈值

赛道现场的光照会变化——室内灯光、窗边阴影、地面反光都会改变灰度分布。
固定阈值在一种光照下调好，换到另一种就会整片失效。动态阈值让二值化自适应。

### 2.3 三个控制实时开销的策略

| 策略 | 宏 | 默认值 | 原因 |
|---|---|---:|---|
| 降采样 | `OTSU_DOWNSAMPLE_STEP_RUN` | 2 | 隔点统计直方图，统计量降为 1/4，阈值几乎不变 |
| 更新间隔 | `OTSU_UPDATE_INTERVAL_RUN` | 6 帧 | 逐帧更新会让阈值随噪声小幅跳动，反而降低二值图稳定性；光照变化是慢过程，不需要每帧重算 |
| 初值 | 上电时对整帧算一次 | — | 避免开头若干帧使用无意义的默认值 128 |

调试模式有一套独立的宏（`OTSU_UPDATE_INTERVAL_DEBUG` 等），便于在菜单中即时观察阈值效果。
降采样版本由 `calculate_otsu_threshold_downsampled()`（`main_cm4.c:1354`）实现。

---

## 3. 二值化

```c
for (int i = 0; i < MT9V03X_H; i++) {
    uint8 *src = mt9v03x_image[i];
    uint8 *dst = bin_image[i];
    for (int j = 0; j < MT9V03X_W; j++) {
        dst[j] = (src[j] >= threshold) ? 255 : 0;
    }
}
```

约定：**`255` 表示赛道（白），`0` 表示赛道外（黑）**。

有一处刻意的分支（`main_cm4.c:1954-1982`）：

```c
bool generate_binary = (g_menu_state == MENU_THRESHOLD_DEBUG) && (g_threshold_debug_show_image != 0);
```

只有在阈值调试菜单下、且需要显示二值图时，才会额外把结果写入全局的
`g_image_binary` 供屏幕绘制。运行模式下 `bin_image` 是唯一的二值图，
省掉一次 180 × 120 的整图拷贝。

---

## 4. 定向滤波

`image_filter()`（`main_cm4.c:2851`）是一个 3 × 3 的**多数表决**滤波，同时具备
填洞与去孤点两种行为：

```c
int white_cnt = 0;
white_cnt += (row_m1[j-1] != 0);   /* 8 个邻居中白点的个数 */
...                                /* 共 8 项，不含中心点 */
uint8 center = row_0[j];
if      (white_cnt >= 6 && center == 0) row_0[j] = 255;   // 孤立黑点 → 填白
else if (white_cnt <= 2 && center != 0) row_0[j] = 0;     // 孤立白点 → 抹黑
```

只有「周围 6 个以上是白」或「周围 2 个以下为白」才改动中心点，处于中间状态时保持原值——
这样不会磨掉真正的边界过渡带。

它有三个限制：

| 限制 | 宏 | 默认值 |
|---|---|---:|
| 总开关 | `IMAGE_FILTER_ENABLE` | 1（开启） |
| 执行间隔 | `IMAGE_FILTER_INTERVAL` | 每 3 帧一次 |
| 行范围 | `IMAGE_FILTER_ROW_START` / `_END` | 第 1–40 行 |

三个限制都是有意的取舍：

- **只在关键行区间处理**：第 1–40 行是算法实际用于判断的远景区间。对更近的行做平滑
  收益很小，却要付出成比例的算力。
- **不是每帧都做**：用更小的平均开销换取更稳定的二值图。八邻域追踪最怕的是孤立噪点，
  而噪点是随机出现的——按固定间隔过滤，等价于用时间维度的冗余换取空间维度的干净。

### 4.1 图像边界锁死

`image_draw_rectan()`（`main_cm4.c:2894`）在滤波之后把二值图的最外圈强制清零：
左右各 3 列、上下各 1 行全部置 0。

目的是**给八邻域追踪设一道墙**。追踪算法沿白色区域外轮廓行走，如果白色区域一直延伸到
图像边缘，边界点会沿着边缘行走并产生大量无效轨迹。锁死边界后，追踪在接近边缘时就会
遇到黑色而停止或转向，行为可预测。

---

## 5. 八邻域边界追踪

### 5.1 为什么不用逐行扫描

逐行扫描（从每一行找左右边界）实现简单，但有两个问题：

1. **抗断裂能力差**：赛道边界被反光或阴影打断一行，该行就会得到错误的边界。
2. **不产出方向信息**：元素识别需要知道边界是「向右上走」还是「向左上走」，
   逐行扫描拿不到这个信息。

八邻域追踪沿边界行走，每一步在 8 个相邻像素里找下一个边界点。它天然抗局部断裂
（少一个点仍然可以继续），并且每一步的**移动方向**就是元素判别需要的信息。

### 5.2 起始点

`get_start_points()`（`main_cm4.c:1502`）在指定行（由 `image_process()` 传入
`MT9V03X_H - 30`，即第 90 行）从图像中心向两侧寻找「白→黑」跳变：

```c
for (int x = MT9V03X_W / 2; x > 1; x--) {
    if (bin_image[start_row][x] == 255 && bin_image[start_row][x - 1] == 0) { *l_x = x; ... }
}
```

选第 90 行而不是最底部，是为了避开图像最下方的噪声与赛道边缘失真。

### 5.3 追踪过程

`search_eight_neighbor()`（`main_cm4.c:1543`）同时驱动左右两侧。

**种子表方向相反**，保证两侧都往图像上方走：

```c
static const int8 seeds_l[8][2] = { {0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} }; // 顺时针
static const int8 seeds_r[8][2] = { {0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1} };  // 逆时针
```

**边界点的判定条件是「前一格为黑、当前格为白」**：

```c
if (bin_image[y1][x1] != 255 && bin_image[y2][x2] == 255) { ... }
```

即沿着白色区域的**外轮廓**行走。

追踪过程维护的数据结构：

| 变量 | 类型 | 作用 |
|---|---|---|
| `points_l` / `points_r` | `uint16[1024][2]` | 左 / 右边界轨迹点 |
| `dir_l` / `dir_r` | `int8[1024]` | 每一步的移动方向（0–7，`-1` 表示未定） |
| `l_dir_sum` / `r_dir_sum` | `uint8[9]` | 方向直方图，供元素判别复用 |
| `points_l_back` / `points_r_back` | `uint16[512][2]` | 回退点，仅用于调试界面绘制（青色） |
| `trace_valid` | `uint8` | 本帧追踪是否成功 |

### 5.4 两个工程细节

**单边冻结**（`main_cm4.c:1573-1598`）

当某一侧在 8 邻域内找不到候选点时，冻结该侧（不再推进、不再记录新点），让另一侧继续爬线。
若该侧一个点都没有，则解除冻结（否则会永久卡住）。这样不会因为一侧短暂丢失边界
而中止整帧追踪。

另外还有一种冻结来源：`l_top_freeze`——某一侧已经到达图像顶行且上一个点也在顶行时冻结，
避免在顶端反复添加重复点。

**接近顶部时停止记录方向**（`main_cm4.c:1627-1654`）

```c
const bool l_near_top = (center_point_l[1] < 5);
...
if (l_near_top) { dir_l[l_data_statics - 1] = -1; }
else            { dir_l[l_data_statics - 1] = (int8)i; l_dir_sum[i]++; }
```

图像顶端往往是赛道消失点，边界在此剧烈转折，方向序列会失真。截断这一段的统计
可以显著降低元素误判率。

### 5.5 资源上限

| 项 | 值 | 说明 |
|---|---:|---|
| 步数上限 | 1,500 | 由调用方作为 `break_flag` 传入，防止死循环 |
| 点集容量 | 1,024 | `l_data_statics < 1024 && r_data_statics < 1024` 时循环才继续 |
| 回退点容量 | 512 | 仅调试用 |

点集到达容量即停止，保证不会越界写。

### 5.6 边界提取

`extract_borders()`（`main_cm4.c:1810`）把 `points_l` / `points_r` 折叠成
按行索引的数组 `left_border[120]` / `right_border[120]`。

约定值（需要注意，容易误读）：

| 值 | 含义 |
|---|---|
| `-1` | 左侧该行丢失 |
| `200` | 右侧该行丢失（不是 `-1`，是历史遗留的哨兵值） |

`200` 这个哨兵值在多处出现，例如 `if (r == 200)`、`if (l < 0 && r == 200)`。
阅读代码时需留意。

---

## 6. 失败兜底：逐行扫描

`fallback_scanline()`（`main_cm4.c:1896`）在八邻域追踪失败时接管。

### 6.1 起点选择

```c
int last_mid = MT9V03X_W / 2;
uint8 *p_bottom_row = mt9v03x_image[MT9V03X_H - 1];
if (p_bottom_row[MT9V03X_W / 2] < threshold) {          // 底行中心是黑的
    if (p_bottom_row[MT9V03X_W / 4] >= threshold)      last_mid = MT9V03X_W / 4;
    else if (p_bottom_row[MT9V03X_W * 3 / 4] >= threshold) last_mid = MT9V03X_W * 3 / 4;
}
```

底行中心为黑说明车已经偏出赛道或正处于急弯，此时从 1/4 或 3/4 处重新找一个白色起点。

### 6.2 逐行扫描与推进

每一行以「白 → 黑 → 黑」的三像素模式确定边界：

```c
for (int j = last_mid; j < MT9V03X_W - 2; j++)
    if (p_row[j] >= threshold && p_row[j+1] < threshold && p_row[j+2] < threshold) { right_border = j; break; }
```

三像素而不是单像素，是为了排除孤立噪点。找到本行中点后，把它作为**下一行（更靠上）**
的搜索起点，并限幅在 `[2, W-3]` 内。

### 6.3 设计取舍

兜底策略的目标**不是「最优」，而是「有结果」**：即使精度下降，也必须输出一条连续的中心线，
使控制层不至于因为连续几帧识别失败而完全失控。

这也是工程同时维护两套数据结构的原因：

| 变量 | 作用域 | 用途 |
|---|---|---|
| `bin_image`、`left_border`、`right_border`、`center_line_i16` | `s_` 语义，文件内 `static` | 算法内部的推理与修线 |
| `g_image_binary`、`g_left_line`、`g_right_line`、`g_center_line` | 全局 | 对外提供，供控制与显示使用 |

`g_*` 是在 `image_process()` 末尾统一从 `s_*` 夹取并写回的（`main_cm4.c:2114-2133`），
因此控制层永远拿到的是**已经限幅过的合法值**。

---

## 7. 中线生成

`main_cm4.c:2071-2094`：

```c
for (int row = 0; row < MT9V03X_H; row++) {
    int16 l = left_border[row];
    int16 r = right_border[row];
    if (l < 0 && r == 200) {                 // 两侧都丢
        center_line_i16[row] = last_mid;     // 沿用上一行的中线
    } else {
        if (l < 0)   l = last_mid - width_from_row(row) / 2;   // 左侧用宽度模型补
        if (r == 200) r = last_mid + width_from_row(row) / 2;  // 右侧用宽度模型补
        center_line_i16[row] = (l + r) / 2;
        last_mid = center_line_i16[row];
    }
}
```

### 7.1 赛道宽度模型

`width_from_row()`（`main_cm4.c:2257`）在顶部宽度与底部宽度之间按行号线性插值：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `g_track_width_top` | 60 px | 图像顶部（远处）的赛道宽度 |
| `g_track_width_bottom` | 180 px | 图像底部（近处）的赛道宽度 |

这利用了透视关系：同一段赛道在图像上方比下方窄。宽度模型只在**单侧丢线**时用于补线，
补出的边界会参与中线计算，但**不会写回 `left_border` / `right_border`**。

`width_init_if_needed()`（`main_cm4.c:2237`）带缓存：只有顶部或底部宽度被改动时才重建
插值表 `width_begin[]` / `width_zzh[]`。

### 7.2 提前相遇补偿

当左右边界在图像中途相遇，说明赛道偏向一侧、前方是不可通行区域。此时需要把中线
强行推到画面边缘，让车头继续朝赛道内侧转（`main_cm4.c:2030-2068`）：

```c
bool meet_happened = (ok && highest_row < (uint8)MT9V03X_H);
```

`highest_row` 是八邻域追踪到达的最高行。若它小于图像高度，说明两侧边界在到达顶部前
就合并了。代码从相遇行向下找到一个可靠的中点作为参考 `ref_mid`：

- `ref_mid < W/2`（赛道偏左）→ 相遇行以上的中线强制取 `2`（画面最左）
- 否则 → 强制取 `W-3`（画面最右）

---

## 8. 帧率测量

`main_cm4.c:296-320` 用 Cortex-M4 的 DWT（Data Watchpoint and Trace）周期计数器测量帧率：

```c
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
...
return (uint32)(DWT->CYCCNT / (SystemCoreClock / 1000U));   // 毫秒
```

`perf_now_ms()` 在图像处理前后各取一次时间戳，差值即为单帧处理耗时。
这比用 PIT 计数更精确，也不需要额外硬件。

---

## 9. 相关文档

- [系统架构](architecture.md)
- [元素识别](element-recognition.md)
- [控制链路与调参](control-and-tuning.md)
