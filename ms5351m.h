#ifndef __MS5351M_H
#define __MS5351M_H
#include "i2c.h"   
#include "sys.h"  

typedef enum {
    ms5351m_PLL_A = 0,
    ms5351m_PLL_B,
} ms5351mPLL_t;

typedef enum {
    ms5351m_R_DIV_1   = 0,
    ms5351m_R_DIV_2   = 1,
    ms5351m_R_DIV_4   = 2,
    ms5351m_R_DIV_8   = 3,
    ms5351m_R_DIV_16  = 4,
    ms5351m_R_DIV_32  = 5,
    ms5351m_R_DIV_64  = 6,
    ms5351m_R_DIV_128 = 7,
} ms5351mRDiv_t;

typedef struct {
    int32_t mult;
    int32_t num;
    int32_t denom;
} ms5351mPLLConfig_t;

/* ---------------- 寄存器定义（仅保留驱动用到的，参考 AN619） ---------------- */
enum {
    ms5351m_REGISTER_3_OUTPUT_ENABLE       = 3,
    ms5351m_REGISTER_16_CLK0_CONTROL       = 16,
    ms5351m_REGISTER_17_CLK1_CONTROL       = 17,
    ms5351m_REGISTER_18_CLK2_CONTROL       = 18,
    ms5351m_REGISTER_26_PLLA_PARAMETERS    = 26,
    ms5351m_REGISTER_34_PLLB_PARAMETERS    = 34,
    ms5351m_REGISTER_42_DIV0_PARAMETERS    = 42,
    ms5351m_REGISTER_50_DIV1_PARAMETERS    = 50,
    ms5351m_REGISTER_58_DIV2_PARAMETERS    = 58,
    ms5351m_REGISTER_165_CLK0_PHASE        = 165,
    ms5351m_REGISTER_166_CLK1_PHASE        = 166,
    ms5351m_REGISTER_167_CLK2_PHASE        = 167,
    ms5351m_REGISTER_177_PLL_RESET         = 177,
    ms5351m_REGISTER_183_XTAL_LOAD         = 183
};

/* ============================================================
   ms5351m 频率范围与组合限制（对照 ms5351m说明书.pdf P32 + 芯片手册 V1.2）
   ------------------------------------------------------------
   【每路范围】(ms5351m_Set 入参，越界自动静默钳位到边界，不报错)
     CLK0/CLK1/CLK2 : 8kHz ~ 200MHz（ms5351m_MIN_FREQ ~ ms5351m_MAX_FREQ）
       - >200MHz -> 钳位到 200MHz
       - <8kHz   -> 钳位到 8kHz（受 DIVn 分频比 <=1800 约束）
       - <1MHz   -> 内部 ×64 + rdiv=64（输出级 2^6 分频）

   【内部硬约束】
     VCO          : 600 ~ 900MHz（PLL_DIV 24~36 @25MHz）
     DIVn 分频比  : 仅 {4,6,8}（整数）或 [8,1800]（分数）
     >150MHz 输出 : 强制 DIVBY4 + INT=1
     >112.5MHz    : 仅允许同时输出 2 路不同时钟（2-PLL 上限）

   【PLL 分配】(3 路独立 MS 小数分频器，2 个 PLL)
     CLK0 -> PLLA
     CLK1 -> CLK0 在场 ? PLLB : PLLA
     CLK2 -> CLK0 在场 ? PLLB : (CLK1 在场 ? PLLB : PLLA)

   【组合限制】
     Case B（CLK0 关闭）: CLK1/CLK2 各占独立 PLL，任意不同频率（8k~200M）都可输出
     Case A（CLK0 在场）: CLK1/CLK2 共享 PLLB；因 DIV1/2 是独立小数分频器，两路可异频：
       - 两路均 <=112.5M  : 任意异频（VCO=900M 分数分频全覆盖，无需凑巧）
       - 有路 >112.5M     : CLK2 用 CLK1 的 VCO 算 DIV，DIV 落在 {4,6,8}∪[8,1800]
                             才可异频，否则强制 CLK2 与 CLK1 同频（保证一定出波）
   ============================================================ */
#define MS5351M_XTAL_FREQ       25000000    // 25MHz 晶振（如需 27MHz 改此宏）
#define MS5351M_MAX_FREQ        200000000   // 输出频率上限 200MHz（ms5351m 规格）
#define MS5351M_MIN_FREQ        8000        // 算法可精确覆盖的下限 8kHz

/* ---------------- 接口（与 MS5352ME 同构） ---------------- */
void ms5351m_write(uint8_t reg, uint8_t value);
void ms5351m_Init(void);
void ms5351m_writePLL(uint8_t baseaddr, const ms5351mPLLConfig_t* pll);
void ms5351m_WriteDivider(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3,
                         uint8_t divBy4, uint8_t rdiv);

/* 核心：按 drive/freq 自动分配 PLL 与 DIV0/1/2，越界静默钳位，void 接口（无返回值） */
void ms5351m_Set(int32_t freq0, uint8_t drive0,
                int32_t freq1, uint8_t drive1,
                int32_t freq2, uint8_t drive2);
#endif
