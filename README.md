# MS5351M 时钟发生器驱动（软件 I2C）
<img width="1169" height="809" alt="image" src="https://github.com/user-attachments/assets/5fb8e8f8-48ec-4124-92b5-8b0ab3805106" />

MS5351M（Si5351A 兼容）三路时钟发生器驱动，基于 STM32 HAL + 软件 I2C。
驱动按硬件规格自动分配 PLL 与分频器，越界频率静默钳位，`void` 接口无返回值。

## 芯片规格（对照数据手册 P32 频率规划）

| 项目 | 规格 |
|---|---|
| 晶振 | 25MHz（`MS5351M_XTAL_FREQ`，可改 27MHz） |
| VCO 范围 | 600 ~ 900MHz（PLL_DIV 24~36 @25MHz） |
| 输出范围 | CLK0/CLK1/CLK2 各 8kHz ~ 200MHz |
| 分频器 | DIV0/DIV1/DIV2 均为**独立小数分频器**（可任意频率） |
| DIVn 分频比 | 仅整数 {4,6,8} 或分数 [8,1800] |
| >150MHz | 强制 DIVBY4 + INT=1 |
| >112.5MHz | 仅允许同时输出 2 路不同时钟（2-PLL 上限） |

## 文件说明

| 文件 | 作用 |
|---|---|
| `Src/ms5351m.c` / `Inc/ms5351m.h` | 驱动主体（频率计算 + PLL/DIV 分配 + 寄存器写入） |
| `Src/i2c.c` / `Inc/i2c.h` | 软件 I2C 底层（写 `my_I2C_sendREG`、读 `my_I2C2_Read_REG`） |

依赖的外部头文件：`sys.h`（HAL/类型）、`rf.h`（引脚宏 `SDA1_*`/`SCL1_*`）、`delay.h`（`Delay_us`）。

## 快速开始

```c
#include "ms5351m.h"

int main(void) {
    ms5351m_Init();                                    // 初始化（输出全部关闭）
    // 三路频率配置：ms5351m_Set(f0,d0, f1,d1, f2,d2)
    ms5351m_Set(57600000, 1,   10000000, 1,   100000000, 1);  // 57.6M / 10M / 100M
    // d = 1/2/3/4 对应驱动电流 3/6/9/12 mA（D1:0 编码）
}
```

- `freq`：期望频率（Hz），越界自动钳位到 [8kHz, 200MHz]，**不会报错、照常配置出波**
- `drive`：驱动强度档位 1~4（`0` = 该路关闭）；`freq=0` 该路也视为关闭
- 配置后 PLL 需要重新锁定，建议等待 ≥10ms 再量波

## PLL 分配规则

```
CLK0 -> PLLA
CLK1 -> CLK0 在场 ? PLLB : PLLA
CLK2 -> CLK0 在场 ? PLLB : (CLK1 在场 ? PLLB : PLLA)
```

## 组合限制（重要）

- **Case B（CLK0 关闭）**：CLK1/CLK2 各占独立 PLL，可输出**任意两个不同频率**（8kHz~200MHz 互不干扰）。
- **Case A（CLK0 在场）**：CLK1/CLK2 共享 PLLB。因 DIV1/2 是独立小数分频器，两路**可以异频**：
  - 两路均 ≤112.5MHz：任意异频（VCO=900MHz 分数分频全覆盖）；
  - 任一路 >112.5MHz：CLK2 用 CLK1 的 VCO 独立算分频比，分频比落在 {4,6,8}∪[8,1800] 才可异频，否则驱动**强制 CLK2 与 CLK1 同频**，保证一定出波。

## 频率边界与越界行为

- `<8kHz` → 钳位到 8kHz（受 DIVn 分频比 ≤1800 约束，算法可精确覆盖的下限）
- `>200MHz` → 钳位到 200MHz
- `<1MHz` → 内部 ×64 + 输出级 rdiv=64（2^6 分频），保证精度

## 使用建议

- 板上 I2C 读取链路（`my_I2C2_Read_REG`）实测曾不稳定，仅作调试回读用；正常配置走 `ms5351m_Set` 写链路即可。
- 若更换晶振，改 `MS5351M_XTAL_FREQ` 宏；VCO/分频算法自动适配。

## 测试方案

`MS5351M_组合测试方案.md`（44 场景，寄存器期望值由与 C 等价的 Python 模型实时计算），覆盖单路/双路/三路、共享异频、强制同频、越界钳位、驱动强度等。
