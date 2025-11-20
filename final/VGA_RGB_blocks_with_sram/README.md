# VGA Image Overlay System - 720p Display with SRAM/BRAM

## 项目概述

本项目实现了一个基于 DE2-115 FPGA 开发板的 VGA 720p 图像显示系统，支持从 SRAM 读取背景图像，从 BRAM 读取前景（zombie）图像，并实现透明像素叠加功能。

## 主要功能

### 1. VGA 720p 显示
- **分辨率**: 1280x720 @ 60Hz
- **时钟频率**: 74.25MHz（通过 PLL 从 50MHz 生成）
- **时序控制**: 符合 VESA 标准的 720p 时序参数

### 2. 背景图像显示
- **数据源**: 外部 SRAM（2MB，16-bit 宽）
- **图像格式**: 1280x720 像素，每个像素 1 byte（8-bit 索引）
- **颜色查找**: 通过 BRAM palette 将 8-bit 索引转换为 RGB565 颜色
- **存储位置**: SRAM 地址 0 开始，按行优先顺序存储

### 3. Zombie 图像叠加
- **数据源**: BRAM（zombie_1x）
- **图像尺寸**: 102x149 像素，每个像素 1 byte（8-bit 索引）
- **颜色查找**: 通过 BRAM palette 将 8-bit 索引转换为 RGB565 颜色
- **透明像素**: 索引值为 `0x00` 的像素被视为透明，显示背景图像
- **位置控制**: 可通过开关控制 zombie 在屏幕上的位置

### 4. Zombie 位置控制（测试功能）
- **控制开关**:
  - `SW[0]`: 向右移动（增加 x1）
  - `SW[1]`: 向左移动（减少 x1）
  - `SW[17]`: 向下移动（增加 y1）
  - `SW[16]`: 向上移动（减少 y1）
- **移动速度**: 约 128 像素/秒
- **边界检查**: 自动限制在屏幕范围内

## 系统架构

### 顶层模块
- **`DE2_115.sv`**: 顶层模块，连接所有子模块和外部接口

### 核心模块

#### 1. `VGA_Controller_720p.sv`
- **功能**: VGA 时序信号生成和像素坐标输出
- **特点**:
  - 使用组合逻辑（`_w`）和时序逻辑（`_r`）分离的编码风格
  - `o_h_count` 和 `o_v_count` 提前 1 个时钟周期输出，补偿 BRAM 读取延迟
  - 输出 `o_active_video` 信号标识有效显示区域

#### 2. `VGA_Image_Overlay_Combined.sv`
- **功能**: 图像叠加处理核心模块
- **特点**:
  - 内部实例化所有 BRAM 模块（background_palette, zombie_1x, zombie_1x_palette）
  - 实现 3 级 pipeline 处理内存延迟：
    - **Stage 0**: 地址计算和注册
    - **Stage 1**: 读取像素索引（SRAM 2 周期延迟，BRAM 1 周期延迟）
    - **Stage 2**: 读取 palette RGB 数据
    - **Stage 3**: RGB565 转 RGB888 并输出
  - 透明像素处理：当 zombie 像素索引为 `0x00` 或不在 zombie 区域内时，显示背景

#### 3. `Zombie_Position_Test_Controller.sv`
- **功能**: Zombie 位置控制模块（用于测试）
- **特点**:
  - 时钟分频器：每秒约 128 次移动 tick
  - 边界检查：防止移出屏幕
  - 互斥控制：同时按下相反方向时不移动

### 数据流 Pipeline

```
VGA Controller (提前 1 cycle)
    ↓
地址计算 (Combinational)
    ↓
Stage 0: 注册地址
    ├─→ SRAM 地址 (2 cycle delay)
    └─→ BRAM 地址 (1 cycle delay)
    ↓
Stage 1: 读取像素索引
    ├─→ SRAM 数据锁存 (sram_dq_r)
    └─→ BRAM 数据 (zombie_bram_data)
    ↓
Stage 2: 读取 Palette RGB
    ├─→ Background RGB565
    └─→ Zombie RGB565
    ↓
Stage 3: RGB565 → RGB888 + 透明处理
    ↓
VGA 输出
```

## 关键技术问题与解决方案

### 问题 1: SRAM 与 BRAM 延迟不同步
**现象**: 背景图像和 zombie 图像出现水平方向像素错位（左移或右移）

**原因**: 
- 外部 SRAM 需要 2 个时钟周期的读取延迟
- BRAM 只需要 1 个时钟周期的读取延迟
- 如果只补偿 1 个周期，会导致数据不同步

**解决方案**:
1. 添加额外的 pipeline stage 来处理 SRAM 的 2 周期延迟
2. 使用 `bg_pixel_byte_addr_r0` 和 `bg_pixel_byte_addr_r1` 两级寄存器
3. 使用 `sram_dq_r` 锁存 SRAM 输出数据
4. 确保背景和 zombie 的像素索引在同一个 pipeline stage 读取

**关键代码**:
```systemverilog
// Stage 0: 注册地址
bg_pixel_byte_addr_r0 <= bg_pixel_byte_addr_w;
bg_pixel_byte_addr_r1 <= bg_pixel_byte_addr_r0;  // SRAM 第二个周期

// Stage 1: 锁存 SRAM 数据
sram_dq_r <= io_bg_sram_dq;  // 锁存 SRAM 输出（2 周期后）

// 读取像素索引（同步）
if (bg_pixel_byte_addr_r1[0] == 1'b0) begin
    bg_pixel_index_r <= sram_dq_r[7:0];
end else begin
    bg_pixel_index_r <= sram_dq_r[15:8];
end
zombie_pixel_index_r <= zombie_bram_data;  // BRAM 1 周期延迟
```

### 问题 2: VGA Controller 输出时序
**现象**: BRAM 数据读取延迟导致像素坐标与数据不匹配

**原因**: BRAM 有 1 个时钟周期的读取延迟，如果 VGA Controller 输出当前周期的坐标，读取到的数据会对应上一个周期的坐标

**解决方案**:
- 在 `VGA_Controller_720p` 中，基于当前周期的计数器计算下一周期的坐标
- `o_h_count` 和 `o_v_count` 提前 1 个时钟周期输出
- 使用 `h_active_count_w` 和 `v_active_count_w` 计算下一周期的坐标

**关键代码**:
```systemverilog
// 基于当前值 h_count_r 计算下一周期的坐标
if (h_count_r >= H_BACK_PORCH && h_count_r < H_BACK_PORCH + H_DISPLAY) begin
    h_active_count_w = h_count_r - H_BACK_PORCH;
end
```

### 问题 3: 代码组织与可维护性
**问题**: 初始代码中调试代码和功能代码混在一起，BRAM 实例化位置不合理

**解决方案**:
1. 将所有 BRAM 实例化移到 `VGA_Image_Overlay_Combined` 模块内部
2. 删除所有调试相关的代码（SRAM_Reader, BRAM_Reader, HEX 显示等）
3. 将 zombie 位置控制功能提取为独立模块 `Zombie_Position_Test_Controller`

## 文件结构

```
VGA_RGB_blocks_with_sram/
├── src/
│   ├── DE2_115/
│   │   └── DE2_115.sv                    # 顶层模块
│   │   
│   ├── VGA_Controller_720p.sv             # VGA 时序控制器
│   ├── VGA_Image_Overlay_Combined.sv     # 图像叠加核心模块
│   └── Zombie_Position_Test_Controller.sv  # 位置控制模块
├── BRAMs/
│   ├── background/
│   │   └── background_palette.v          # 背景调色板 BRAM
│   └── zombies/
│       ├── zombie_1x.v                   # Zombie 图像 BRAM
│       └── zombie_1x_palette.v           # Zombie 调色板 BRAM
├── vga_clock/                             # PLL 时钟生成
└── README.md                              # 本文件
```

## 数据格式

### 背景图像
- **SRAM 存储**: `background.bin`
- **格式**: 1280x720 像素，每个像素 1 byte
- **地址映射**: `address = y * 1280 + x`（字节地址）
- **SRAM 接口**: 16-bit 宽，需要根据字节地址的低位选择高低字节

### Palette 格式
- **BRAM 存储**: `background_palette.mif`, `zombie_1x_palette.mif`
- **格式**: 256 个条目，每个条目 16-bit RGB565
- **颜色转换**: RGB565 → RGB888
  - R: `{rgb565[15:11], rgb565[15:13]}` (5 bits → 8 bits)
  - G: `{rgb565[10:5], rgb565[10:9]}` (6 bits → 8 bits)
  - B: `{rgb565[4:0], rgb565[4:2]}` (5 bits → 8 bits)

### Zombie 图像
- **BRAM 存储**: `zombie_1x.mif`
- **格式**: 102x149 像素，每个像素 1 byte
- **透明像素**: 索引值 `0x00` 表示透明

## 使用说明

### 硬件连接
- **开发板**: Altera DE2-115
- **VGA 输出**: 连接到 VGA 显示器
- **SRAM**: 外部 SRAM 芯片（2MB，16-bit）
- **开关控制**:
  - `KEY[1]`: 复位（低电平有效）
  - `SW[0]`: Zombie 向右移动
  - `SW[1]`: Zombie 向左移动
  - `SW[17]`: Zombie 向下移动
  - `SW[16]`: Zombie 向上移动

### 编译与下载
1. 使用 Quartus Prime 打开项目文件 `VGA_SRAM.qpf`
2. 确保所有 BRAM IP 核已正确生成
3. 编译项目
4. 将生成的 `.sof` 文件下载到 FPGA

### 数据准备
1. 将 `background.bin` 写入 SRAM（地址 0 开始）
2. 确保 BRAM 初始化文件（`.mif`）已正确配置

## 版本历史

### Version 1.0 (当前版本)
- ✅ 实现 VGA 720p 显示
- ✅ 实现背景图像从 SRAM 读取并显示
- ✅ 实现 Zombie 图像从 BRAM 读取并叠加
- ✅ 实现透明像素处理
- ✅ 修复 SRAM/BRAM 延迟同步问题
- ✅ 实现 Zombie 位置控制功能
- ✅ 代码重构：模块化设计，分离组合逻辑和时序逻辑
- ✅ 清理调试代码，优化代码结构

## 待改进项目

- [ ] 支持多个 Zombie 同时显示
- [ ] 添加动画功能（多帧图像切换）
- [ ] 优化移动速度控制（可配置）
- [ ] 添加碰撞检测功能
- [ ] 支持图像缩放功能

## 技术要点总结

1. **Pipeline 设计**: 正确处理多级内存延迟，确保数据同步
2. **时序补偿**: VGA Controller 提前输出坐标，补偿 BRAM 延迟
3. **代码风格**: 使用 `_w`（组合逻辑）和 `_r`（时序逻辑）命名约定，提高可读性
4. **模块化设计**: 功能模块独立，便于测试和维护
5. **边界检查**: 防止图像移出屏幕范围

## 参考资料

- VESA Video Timing Standards
- Altera DE2-115 User Manual
- SystemVerilog Coding Guidelines

---

**最后更新**: 2024
**作者**: 潘猴猴

