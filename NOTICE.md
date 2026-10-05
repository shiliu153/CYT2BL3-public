# 第三方组件与权利归属说明

本工程是**衍生作品**：应用层与控制算法为本人编写，底层依赖逐飞科技的第三方开源库，
逐飞库之下又依赖芯片厂商的 SDK。按各自许可要求，出处与改动情况如实列明如下。

> **请先读第 3 节。** 本工程依赖的芯片厂商 SDK **不是开源软件**，其许可并未授予公开再分发权利。
> 本文档把这一事实显式写出来，供使用者自行判断。

---

## 1. 逐飞科技 CYT2BL3 开源库

| 项 | 值 |
|---|---|
| 路径 | `libraries/zf_common/`、`libraries/zf_driver/`、`libraries/zf_device/`、`libraries/zf_components/` |
| 版本 | **V3.3.3**（见 `libraries/doc/version.txt`） |
| 版权 | Copyright (C) 2022 SEEKFREE 成都逐飞科技有限公司 |
| 许可 | **GNU General Public License v3.0 or later** |
| 许可正文 | `libraries/LICENSE`；英文声明另见 `libraries/doc/GPL3_permission_statement.txt` |
| 上游 | <https://seekfree.taobao.com/> |

所有文件内的版权声明均原样保留。

### 1.1 本工程对该库的改动（逐文件披露）

以本工程首次导入该库的提交为基准，改动如下：

| 文件 | 改动性质 | 说明 |
|---|---|---|
| `zf_device/zf_device_mt9v03x.h` | **功能参数** | 图像宽度 `188 → 180`；曝光时间上限 `500 → 400`；帧率上限 `50 → 100`；总钻风摄像头 IIC 的 SCL/SDA 引脚对调（`P11_1` ↔ `P11_2`） |
| `zf_device/zf_device_imu660ra.h` | 仅字符编码 | GBK → UTF-8，解码后逐字符一致 |
| `zf_device/zf_device_imu963ra.h` | 仅字符编码 | 同上 |
| `zf_device/zf_device_ips200.h` | 仅字符编码 | 同上 |
| `zf_driver/zf_driver_flash.c` | 仅字符编码 | 同上 |
| `zf_driver/zf_driver_flash.h` | 仅字符编码 | 同上 |
| `zf_common/zf_common_headfile.h` | **新增包含** | 增加 `#include "zf_device_imu660rb.h"` 与 `#include "pid.h"` 两行 |
| `zf_common/zf_common_debug.h` | 见 §1.2 | 中文注释曾被误转码破坏，本次发布已恢复 |

「仅字符编码」表示把这些文件按其原始编码解码后，与上游原文**逐字符相同**——没有任何代码或注释内容的改动，
差异全部来自字节层面的编码转换。

### 1.2 本工程新增在逐飞库目录下的文件

| 文件 | 行数 | 说明 |
|---|---:|---|
| `zf_common/pid.h` | 22 | `PID_t` 结构体与 `menu_state_t` 菜单状态枚举。放在 `zf_common/` 下是为了复用该目录已在 IAR 工程的包含路径中。**无第三方版权声明，属本工程自研代码。** |
| `zf_device/zf_device_imu660rb.c` | 115 | 逐飞 IMU660RB 模块驱动 |
| `zf_device/zf_device_imu660rb.h` | 48 | 同上 |

后两个文件是逐飞官方的 IMU660RB 驱动（API 风格、寄存器定义与 `zf_device_imu660ra.h` 同源），
但其文件头**未附版权声明**（上游如此）。本工程按逐飞开源库的 GPL-3.0 处理，不主张任何权利。

### 1.3 编码说明

逐飞上游源文件为 **GBK** 编码。本工程保留该编码未做转换（`libraries/` 下绝大多数 `.c/.h` 仍为 GBK），
以便后续跟进上游更新。本工程自研代码（`project/user/`）与本文档为 **UTF-8**。

### 1.4 关于两个文件的注释恢复（可复现）

在源工程的开发过程中，有两次提交因编辑器误判编码而破坏了两个文件的中文注释：

| 文件 | 破坏形式 | 破坏后的状态 |
|---|---|---|
| `zf_common/zf_common_headfile.h` | GBK 字节被当作 UTF-8 读入后另存 | 520 个 U+FFFD 替换字符，正文中文全部不可读 |
| `zf_common/zf_common_debug.h` | 非 ASCII 字符被替换为 `?` | 1,290 个 `?` 字符，中文全部不可读 |

已核对确认**破坏仅发生在注释中，代码部分未受影响**（逐行比对非注释行：`headfile.h` 除新增的两行
`#include` 外完全一致；`debug.h` 的 `#define` 名称与取值逐字符一致）。

本发布版本按 git 历史中**首次导入时的原始字节**恢复这两个文件，并重新施加源工程真正做过的改动
（`headfile.h` 的两行 `#include`）。恢复结果与上游 V3.3.3 原文一致。可复现的验证方式：

```python
# 取首次导入版本中 zf_common_headfile.h 的原始字节，
# 按 UTF-8 解码并对非法字节使用 replace 策略，即可逐字节重现被破坏后的文件内容
import subprocess
raw = subprocess.run(["git", "show", "<首次导入提交>:libraries/zf_common/zf_common_headfile.h"],
                     capture_output=True).stdout
corrupted = raw.decode("utf-8", errors="replace")   # 与源工程当前内容一致（除新增的两行 #include）
```

> 源工程本身未做修改，这两个文件在源工程中仍为损坏状态。

---

## 2. 芯片厂商 SDK：英飞凌 / 赛普拉斯 Traveo II SDK

| 项 | 值 |
|---|---|
| 路径 | `libraries/sdk/`（含 `common/` 与 `tviibh4m/`） |
| 文件数 | 367 |
| 版权 | Copyright Cypress Semiconductor Corporation / Infineon Technologies AG |
| 上游 | Infineon TRAVEO™ T2G Sample Driver Library |
| 本工程改动 | **无**，全部文件保持原始字节 |

### 2.1 许可状态（重要）

该 SDK 的**大部分源文件不是以开源许可发布的**。逐字引用两种文件头：

- **232 个 `.c/.h`** 文件写的是：

  > You may use this file only in accordance with the license, terms, conditions, disclaimers, and limitations in the end user license agreement accompanying the software package with which this file was provided.

- 其中 `common/src/drivers/mpu/cy_mpu.h`、`common/src/drivers/systick/cy_systick.h` 给出的是完整条款：

  > Cypress hereby grants to licensee a **personal, non-exclusive, non-transferable** license to copy, use, modify, create derivative works of, and compile the Cypress Source Code … for the sole purpose of creating custom software in support of licensee product to be used only in conjunction with a Cypress integrated circuit as specified in the applicable agreement. **Any reproduction, modification, translation, compilation, or representation of this software except as specified above is prohibited without the express written permission of Cypress.**

**这两段文字没有授予公开再分发（redistribution）的权利。** 本仓库为保持工程「克隆即可编译」的完整性而
原样包含了该目录，但使用者应当知悉：这部分代码的权利不属于本工程作者，其使用条件以上游最终用户许可协议为准。
若你需要严格合规的再分发，请从 Infineon 官方渠道获取 SDK 并自行确认许可条款；也可以删除
`libraries/sdk/` 后从官方 SDK 中复制同名目录回来。

### 2.2 其中的开源组件

`libraries/sdk/` 内还包含两组**真正的开源**代码，与上述专有文件不同：

| 组件 | 路径 | 许可 |
|---|---|---|
| FreeRTOS 内核与 FreeRTOS+TCP | `sdk/common/src/rtos/`、`sdk/common/src/ethstack/` | MIT（41 个文件带 `SPDX-License-Identifier: MIT`；另见 `sdk/common/src/rtos/license.txt`） |
| ARM CMSIS | `sdk/common/hdr/cmsis/` | Apache-2.0（见 `sdk/common/hdr/cmsis/LICENSE.txt`） |

### 2.3 预编译静态库

`libraries/sdk/` 下有 6 个预编译二进制（GCC / GHS / IAR 三套工具链的 crypto 客户端与服务端库）：

```
tviibh4m/src/drivers/crypto/libs/libcrypto_client_cm4_gcc.a
tviibh4m/src/drivers/crypto/libs/libcrypto_client_cm4_ghs.a
tviibh4m/src/drivers/crypto/libs/libcrypto_client_cm4_iar.a
tviibh4m/src/drivers/crypto/libs/libcrypto_server_cm0plus_gcc.a
tviibh4m/src/drivers/crypto/libs/libcrypto_server_cm0plus_ghs.a
tviibh4m/src/drivers/crypto/libs/libcrypto_server_cm0plus_iar.a
```

这些是上游随 SDK 一起分发的产物，本工程未做修改。其中 `.a` 文件内含上游构建机器的调试路径字符串
（形如 `C:\Users\<上游开发者>\Desktop\work\TraveoII\...`），属上游原始内容。

---

## 3. 本工程自研部分

| 路径 | 说明 |
|---|---|
| `project/user/main_cm4.c` | CM4 主程序：图像处理、元素识别、控制链路、菜单、参数持久化 |
| `project/user/cm4_isr.c` | PIT 中断服务函数：速度环与转向环 |
| `project/iar/` | IAR 工程配置（不含 `settings/`，见下） |
| `docs/` | 本工程的说明文档 |
| `tools/` | 发布前自检脚本 |
| 根目录 `.gitignore`、`NOTICE.md`、`CONTRIBUTING.md`、`README.md` | 工程与许可文件 |

许可：**GNU General Public License v3.0 or later**（`LICENSE`）。
原因是自研代码静态链接了逐飞 GPL-3.0 库，整体按 GPL-3.0 发布。

### 3.1 未纳入本仓库的内容

为保持仓库整洁与安全，以下内容**不在公开范围内**：

| 内容 | 原因 |
|---|---|
| `project/iar/Debug_m4/` | IAR 构建产物（目标文件、可执行文件、browse 数据库），其中含构建机的绝对路径 |
| `project/iar/settings/`、`project/iar/project_config/settings/` | IAR 个人会话状态，内容与具体机器绑定并含绝对路径。IAR 首次打开工程时会自动重建 |
| `project/doc/` | 个人编写的讲解稿与变量速查表，不属于开源内容 |

---

## 4. 许可兼容性提示

需要说明一个既存的事实：逐飞 CYT2BL3 开源库以 **GPL-3.0** 发布，而其底层依赖的
Traveo II SDK 带有**专有许可**。GPL-3.0 与专有许可链接所构成的组合作品，其许可状态在上游
本身即存在张力——这不是本工程引入的问题，也不是本工程能够解决的。本工程所做的只是：
保留逐飞库的 GPL 声明、保留 SDK 的原有版权头、并把这一情况如实写明。

---

## 5. 问题反馈

若你是上述任一第三方组件的权利人，认为本仓库的收录方式不妥，请通过仓库 Issue 联系，我会立即处理。
