# 构建与烧录

---

## 1. 环境要求

| 项 | 版本 |
|---|---|
| IDE | **IAR Embedded Workbench for ARM 9.40.1** |
| 目标器件 | Infineon CYT2BL3（TRAVEO™ T2G Body High） |
| 调试器 | 支持 SWD 的调试探针 |

工程按 IAR 9.40.1 建立（逐飞库各文件头的「开发环境」一栏亦标注该版本）。
更低版本可能可以打开，但未经验证。

> **本仓库未在命令行环境验证编译。** IAR 是仅图形界面的商业工具链，本次开源整理过程
> 未执行任何编译。因此仓库中不含 `.out` / `.hex` / `.map` 等构建产物，
> README 与文档中的所有参数值均直接取自源码，而不是取自实测报告。

---

## 2. 工程文件

| 文件 | 说明 |
|---|---|
| `project/iar/cyt2bl3.eww` | IAR 工作区，**从这里打开** |
| `project/iar/project_config/cyt2bl3.ewp` | 工程文件（源文件列表、包含路径、编译选项） |
| `project/iar/project_config/cyt2bl3.ewd` | 调试器配置 |
| `project/iar/project_config/cyt2bl3.ewt` | 测试 / C-STAT 配置 |
| `project/iar/project_config/cyt2bl3.ewx` | 扩展配置 |
| `project/iar/icf/linker_directives_tviibe4m.icf` | 链接脚本 |
| `project/iar/删除临时文件IAR.bat` | 清理脚本（见 §5） |

---

## 3. 构建步骤

1. 用 IAR 打开 `project/iar/cyt2bl3.eww`
2. 在工程下拉框中选择 **`Debug_m4`** 配置
3. `Project → Rebuild All`
4. 连接调试器，`Download and Debug` 烧录

首次打开时 IAR 会在 `project/iar/settings/` 与 `project/iar/project_config/settings/`
下生成本机相关的会话文件。**这两个目录不在本仓库中**（原因见 `NOTICE.md` §3.1），
IAR 会自动重建，无需手动处理。

---

## 4. 编码设置（重要）

本工程的源文件存在两种编码：

| 路径 | 编码 |
|---|---|
| `libraries/`（逐飞库） | **GBK** |
| `project/user/`（自研代码） | UTF-8 |

**请勿让编辑器自动转换 `libraries/` 的编码。** 本工程在开发过程中就有两个文件因此损坏：

| 文件 | 后果 |
|---|---|
| `libraries/zf_common/zf_common_headfile.h` | 520 个 U+FFFD，中文注释全部不可读 |
| `libraries/zf_common/zf_common_debug.h` | 1,290 个 `?`，中文注释全部不可读 |

本仓库中的这两个文件已按 git 历史中的原始字节恢复（过程与可复现的验证方式见
`NOTICE.md` §1.4），但**再次用错误的编码保存仍会重新破坏它们**。

### 4.1 建议配置

**IAR**：`Tools → Options → Editor → Encoding`，设为系统默认或为 `libraries/` 单独指定 GBK。

**VS Code / CLion**：关闭 `files.autoGuessEncoding`，并在工作区设置中为 `libraries/**`
指定 `files.encoding: gb18030`。本仓库的 `.vscode/settings.json` 已把
`zf_common_headfile.h` 关联为 C 语言文件，可在此基础上补充编码设置。

**Git**：仓库根目录的 `.gitattributes` **未**设置编码转换规则，请勿添加
`working-tree-encoding`，否则会改变 `libraries/` 下的字节并破坏与上游的一致性。

---

## 5. 清理构建产物

IAR 的 `Debug_m4/` 目录在完整构建后可达数百 MB。工程自带清理脚本
`project/iar/删除临时文件IAR.bat`，它删除的目录与文件包括：

```
Debug/  Debug_m4/  Debug_m0/  settings/  settings_m0/  settings_m7_0/  settings_m7_1/
program/nor_sdram_zf_dtcm/  project_config/settings/  project_config/rev_a/
*.crun  *.dbgdt  *.ps1  *.xcl  *.reggroups
```

> 该脚本用 `rmdir /s /q` 直接删除，**不可恢复**。运行前请确认当前目录没有需要保留的内容。

这些路径已在 `.gitignore` 中排除，正常情况下不会被误提交。

---

## 6. 常见问题

### 6.1 打开工程后提示找不到源文件

`cyt2bl3.ewp` 中的源文件路径是**相对于工程文件的相对路径**。若把工程目录挪到别处，
只要保持 `project/` 与 `libraries/` 的相对位置不变，就不需要修改配置。

### 6.2 链接报错：找不到 `pit0_chN_isr` 等符号

`cm4_isr.c` 中有 22 个空函数体的中断入口占位（15 个 PIT 通道中的 13 个、
部分 UART 与 GPIO 外部中断）。这些不能删除——逐飞库的 `pit_us_init()` 等接口会引用
对应符号，缺失会导致链接失败。

### 6.3 库函数未定义（`zf_device_config.*`）

`libraries/zf_device/` 下有逐飞预编译的 `zf_device_config.a` 与 `zf_device_config.lib`。
该库按工程实际启用的设备模块裁剪，若在 `zf_common_headfile.h` 中启用了新的设备
（如新增一个 IMU 型号），可能需要更新该库文件或改用源码编译。

### 6.4 中文注释显示为乱码

见 §4。**不要用编辑器「修正」它** —— 用正确编码重新打开即可，保存会破坏内容。

### 6.5 舵机在上电瞬间猛打角

工程已在 `main()` 中把 `g_servo_duty` 先设为中位值再调用 `pwm_init()`
（`main_cm4.c:409-410`）。若仍然出现，检查舵机的中位 duty 标定值是否正确。

---

## 7. 相关文档

- [引脚与资源](pinout.md)
- [控制链路与调参](control-and-tuning.md)
- [发布说明](PUBLICATION.md)
