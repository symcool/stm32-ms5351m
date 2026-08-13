#include "ms5351m.h"
#include "i2c.h"
#include "sys.h"
#include "rf.h"

/* ================= 底层 ================= */

void ms5351m_write(uint8_t reg, uint8_t value) {
    my_I2C_sendREG(reg, value);
}

/* MS5351M 专用初始化 */
void ms5351m_Init(void) {
    ms5351m_write(ms5351m_REGISTER_3_OUTPUT_ENABLE, 0xFF);
    ms5351m_write(ms5351m_REGISTER_16_CLK0_CONTROL, 0x80);
    ms5351m_write(ms5351m_REGISTER_17_CLK1_CONTROL, 0x80);
    ms5351m_write(ms5351m_REGISTER_18_CLK2_CONTROL, 0x80);
    /* Reg183 晶振负载：实际使用有源晶振（时钟灌 XA、XB 悬空），内部负载电容无实际作用；
       统一按 PDF 推荐 10pF（XTAL_CL=11b，0xC0|0x12=0xD2，与 MS5352ME 一致），D5:0 必须写 010010b */
    ms5351m_write(ms5351m_REGISTER_183_XTAL_LOAD, 0xC0 | 0x12);
}

/* ================= 寄存器写入 ================= */

/* 写 8 字节 PLL 参数（Reg26/34）
 * 由 pll 配置(mult,num,denom)按手册公式算出 P1/P2/P3 并突发写入。*/
void ms5351m_writePLL(uint8_t baseaddr, const ms5351mPLLConfig_t* pll) {
    int64_t a = pll->mult;
    int64_t b = pll->num;
    int64_t c = pll->denom;
    int64_t P1 = 128 * a + (128 * b) / c - 512;
    int64_t P2 = 128 * b - c * ((128 * b) / c);
    int64_t P3 = c;
    ms5351m_write(baseaddr,   (P3 >> 8) & 0xFF);
    ms5351m_write(baseaddr+1, P3 & 0xFF);
    ms5351m_write(baseaddr+2, (P1 >> 16) & 0x03);
    ms5351m_write(baseaddr+3, (P1 >> 8) & 0xFF);
    ms5351m_write(baseaddr+4, P1 & 0xFF);
    ms5351m_write(baseaddr+5, ((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0x0F));
    ms5351m_write(baseaddr+6, (P2 >> 8) & 0xFF);
    ms5351m_write(baseaddr+7, P2 & 0xFF);
}

/* 写分频器参数（DIV0->Reg42 / DIV1->Reg50 / DIV2->Reg58）
 * divBy4 : DIVn 比值=4 时的 DIVBY4 标志（D3:2）
 * rdiv   : 输出级 OUTn_DIV 的 2^N 指数 N（D6:4）*/
void ms5351m_WriteDivider(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3,
                         uint8_t divBy4, uint8_t rdiv) {
    ms5351m_write(baseaddr,   (P3 >> 8) & 0xFF);
    ms5351m_write(baseaddr+1, P3 & 0xFF);
    ms5351m_write(baseaddr+2, ((P1 >> 16) & 0x03) |
                             ((divBy4 & 0x03) << 2) |
                             ((rdiv & 0x07) << 4));
    ms5351m_write(baseaddr+3, (P1 >> 8) & 0xFF);
    ms5351m_write(baseaddr+4, P1 & 0xFF);
    ms5351m_write(baseaddr+5, ((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0x0F));
    ms5351m_write(baseaddr+6, (P2 >> 8) & 0xFF);
    ms5351m_write(baseaddr+7, P2 & 0xFF);
}

/* 输出级分频（rdiv）计算：freq<1MHz 用 ×64 + R_DIV_64，否则 R_DIV_1
 * （与 MS5352 的 CalcDiv0 不同点：MS5351 三路都是小数分频器，均走此逻辑） */
static uint8_t ms5351m_CalcRDiv(int32_t* freq) {
    if (*freq < 1000000) {
        *freq *= 64;
        return ms5351m_R_DIV_64;
    }
    return ms5351m_R_DIV_1;
}

/* DIVn（小数分频器）参数计算：由输出频率反算分频比与 PLL 倍频。
 * VCO 硬范围 600~900MHz（PLL_DIV 24~36@25MHz）：
 *   - 低频（fpre<=81M）: VCO 固定 900MHz（整数倍频 a=36），DIV 分数 [8,1800]
 *   - 高频（fpre>81M） : 整数档 x∈{4,6,8}，VCO=x*fpre∈[648M,900M]
 * Fvco 为输出参数（可传 NULL）：返回本路选用的 VCO 频率，供共享同一 PLL 的另一路复用。*/
static void ms5351m_CalcDiv0(int32_t freq, ms5351mPLLConfig_t* pll,
                            int32_t* P1, int32_t* P2, int32_t* P3,
                            uint8_t* divBy4, uint8_t* rdiv, uint8_t* isInt,
                            int32_t* Fvco) {
    /* 钳位：下限 8kHz（DIVn 分频比 <=1800 的硬约束），上限 200MHz（MS5351M 规格） */
    if (freq < MS5351M_MIN_FREQ) freq = MS5351M_MIN_FREQ;
    else if (freq > MS5351M_MAX_FREQ) freq = MS5351M_MAX_FREQ;

    *rdiv = ms5351m_CalcRDiv(&freq);

    const int32_t Fxtal = MS5351M_XTAL_FREQ;
    int32_t a, b, c, x, y, z, t;

    if (freq < 81000000) {
        /* 适用 0.5~112.5MHz 区间（超过 81MHz 误差 >6Hz） */
        a = 36; /* PLL 跑 900MHz（36 x 25M） */
        b = 0;
        c = 1;
        int32_t Fpll = 900000000;
        x = Fpll / freq;
        t = (freq >> 20) + 1;
        y = (Fpll % freq) / t;
        z = freq / t;
        if (Fvco) *Fvco = Fpll;
    } else {
        /* 适用 75~160MHz 区间：整数档 4/6/8 */
        if (freq >= 150000000) {
            x = 4;
        } else if (freq >= 100000000) {
            x = 6;
        } else {
            x = 8;
        }
        y = 0;
        z = 1;

        int32_t numerator = x * freq;
        a = numerator / Fxtal;
        t = (Fxtal >> 20) + 1;
        b = (numerator % Fxtal) / t;
        c = Fxtal / t;
        if (Fvco) *Fvco = numerator;
    }

    pll->mult = a;
    pll->num = b;
    pll->denom = c;

    /* 编码 P1/P2/P3（与 SetupOutput 原逻辑一致：仅 div==4 走 DIVBY4 特例，
       其余统一公式，整数比时 P3 保持分母 z——MS5351 未做 MS5352 的 P3=1 加固） */
    if (x == 4) {
        /* DIVBY4 特例（AN619 4.1.3）：P1=0,P2=0,P3=1 */
        *P1 = 0;
        *P2 = 0;
        *P3 = 1;
        *divBy4 = 0x3;
    } else {
        *P1 = 128 * x + (128 * y) / z - 512;
        *P2 = (128 * y) % z;
        *P3 = z;
        *divBy4 = 0;
    }
    *isInt = (y == 0) || (x == 4);   /* 整数模式：num==0 或 div==4 */
}

/* 用给定 VCO 为共享 PLL 的第二路计算分频参数（如 CLK0 在场时 CLK1/CLK2 共 PLLB）。
 * 分频比必须落在 {4,6,8}(y==0) 或 [8,1800] 内（MS5351M说明书 P32 3.2 硬约束），
 * 否则返回 0（调用方应强制该路同频）。
 * 注意：div==4 时芯片走 DIVBY4（VCO/4 精确整数），故 x==4 且 y!=0（非整除）判非法。*/
static int ms5351m_CalcDivForVCO(int32_t freq, int32_t Fvco,
                                int32_t* P1, int32_t* P2, int32_t* P3,
                                uint8_t* divBy4, uint8_t* rdiv, uint8_t* isInt) {
    if (freq < MS5351M_MIN_FREQ) freq = MS5351M_MIN_FREQ;
    else if (freq > MS5351M_MAX_FREQ) freq = MS5351M_MAX_FREQ;

    *rdiv = ms5351m_CalcRDiv(&freq);

    int32_t x = Fvco / freq;
    int32_t t = (freq >> 20) + 1;
    int32_t y = (Fvco % freq) / t;
    int32_t z = freq / t;

    int ok;
    if (y == 0) {
        ok = (x == 4) || (x == 6) || (x == 8) || (x >= 8 && x <= 1800);
    } else {
        ok = (x >= 8 && x <= 1800);   /* 分数比下限 8；x==4/6/8 的分数形式非法（DIVBY4 只支持精确 4） */
    }
    if (!ok) return 0;

    /* 编码 P1/P2/P3（与 ms5351m_CalcDiv0 相同规则，仅 div==4 走 DIVBY4 特例） */
    if (x == 4) {
        *P1 = 0;
        *P2 = 0;
        *P3 = 1;
        *divBy4 = 0x3;
    } else {
        *P1 = 128 * x + (128 * y) / z - 512;
        *P2 = (128 * y) % z;
        *P3 = z;
        *divBy4 = 0;
    }
    *isInt = (y == 0) || (x == 4);
    return 1;
}

/* 写某路输出控制寄存器（Reg16/17/18）
 * pll   : DIVn_SRC（0=PLLA，1=PLLB）
 * drive : D1:0 驱动能力（0~3）
 * isInt : D6 DIVn_INT（1=整数模式）*/
static void ms5351m_WriteClkControl(uint8_t output, uint8_t pll,
                                   uint8_t drive, uint8_t isInt) {
    uint8_t reg = 16 + output;
    uint8_t ctrl = 0x0C;                 /* 时钟不反相、输出上电，D3:2=11 选 DIVn */
    ctrl |= (pll & 0x01) << 5;           /* D5 : DIVn_SRC */
    ctrl |= (isInt & 0x01) << 6;         /* D6 : DIVn_INT */
    ctrl |= (drive & 0x03);              /* D1:0 : 驱动能力 */
    ms5351m_write(reg, ctrl);
}

/* 核心：频率设置与 PLL/DIV 分配（与 MS5352M 修改版同构）
 * 资源约束（MS5351M说明书 P32 频率规划）：
 *   - 三路 CLK0/1/2 均使用独立的小数分频器 DIV0/1/2，输出 8kHz~200MHz；
 *   - 仅 2 个 PLL（VCO 600~900MHz），3 路分频器：
 *       CLK0 -> PLLA；CLK1 -> 有 CLK0 在场借 PLLB 否则 PLLA；
 *       CLK2 -> 有 CLK0 在场借 PLLB，否则有 CLK1 借 PLLB，否则 PLLA。
 *   - 共享协调：CLK1/CLK2 共享同一 PLL 时（仅 CLK0 在场时发生），
 *     CLK2 用 CLK1 的 VCO 独立算分频比（DIV1/2 是独立小数分频器，可异频）；
 *     分频比不在 {4,6,8}∪[8,1800] 内则强制 CLK2 与 CLK1 同频，保证一定出波。
 * 越界频率一律静默钳位到硬件边界并照常配置；本函数为 void 接口，不返回任何状态。*/
void ms5351m_Set(int32_t freq0, uint8_t drive0,
                int32_t freq1, uint8_t drive1,
                int32_t freq2, uint8_t drive2)
{
    uint8_t en0 = (drive0 > 0) && (freq0 > 0);
    uint8_t en1 = (drive1 > 0) && (freq1 > 0);
    uint8_t en2 = (drive2 > 0) && (freq2 > 0);
    uint8_t sd0 = (drive0 > 0) ? (drive0 - 1) : 0;
    uint8_t sd1 = (drive1 > 0) ? (drive1 - 1) : 0;
    uint8_t sd2 = (drive2 > 0) ? (drive2 - 1) : 0;
    if (sd0 > 3) sd0 = 3;
    if (sd1 > 3) sd1 = 3;
    if (sd2 > 3) sd2 = 3;

    /* 越界静默钳位（参考 MS5352M 修改版：钳位到硬件边界，不报错、照常配置） */
    int32_t f0 = freq0, f1 = freq1, f2 = freq2;
    if (en0) { if (f0 < MS5351M_MIN_FREQ) f0 = MS5351M_MIN_FREQ; else if (f0 > MS5351M_MAX_FREQ) f0 = MS5351M_MAX_FREQ; }
    if (en1) { if (f1 < MS5351M_MIN_FREQ) f1 = MS5351M_MIN_FREQ; else if (f1 > MS5351M_MAX_FREQ) f1 = MS5351M_MAX_FREQ; }
    if (en2) { if (f2 < MS5351M_MIN_FREQ) f2 = MS5351M_MIN_FREQ; else if (f2 > MS5351M_MAX_FREQ) f2 = MS5351M_MAX_FREQ; }

    /* PLL 分配（同 MS5352M 修改版） */
    uint8_t pll1 = en0 ? ms5351m_PLL_B : ms5351m_PLL_A;
    uint8_t pll2 = en0 ? ms5351m_PLL_B : (en1 ? ms5351m_PLL_B : ms5351m_PLL_A);

    /* ================= 写寄存器 ================= */
    uint8_t usedPllA = 0, usedPllB = 0;
    int32_t Fvco1 = 0;

    /* CLK0 -> DIV0（PLLA，独立） */
    if (en0) {
        ms5351mPLLConfig_t pll0;
        int32_t P1=0, P2=0, P3=1; uint8_t divBy4=0, rdiv0=0, isInt0=0;
        ms5351m_CalcDiv0(f0, &pll0, &P1, &P2, &P3, &divBy4, &rdiv0, &isInt0, 0);
        ms5351m_WriteDivider(ms5351m_REGISTER_42_DIV0_PARAMETERS, P1, P2, P3, divBy4, rdiv0);
        ms5351m_writePLL(ms5351m_REGISTER_26_PLLA_PARAMETERS, &pll0);
        usedPllA = 1;
        ms5351m_WriteClkControl(0, ms5351m_PLL_A, sd0, isInt0);
        ms5351m_write(ms5351m_REGISTER_165_CLK0_PHASE, 0);
    } else {
        ms5351m_write(ms5351m_REGISTER_16_CLK0_CONTROL, 0x80);
    }

    /* CLK1 -> DIV1（PLL 见分配） */
    if (en1) {
        ms5351mPLLConfig_t pll1cfg;
        int32_t P1=0, P2=0, P3=1; uint8_t divBy4=0, rdiv1=0, isInt1=0;
        ms5351m_CalcDiv0(f1, &pll1cfg, &P1, &P2, &P3, &divBy4, &rdiv1, &isInt1, &Fvco1);
        ms5351m_WriteDivider(ms5351m_REGISTER_50_DIV1_PARAMETERS, P1, P2, P3, divBy4, rdiv1);
        ms5351m_writePLL((pll1 == ms5351m_PLL_B) ? ms5351m_REGISTER_34_PLLB_PARAMETERS
                                               : ms5351m_REGISTER_26_PLLA_PARAMETERS, &pll1cfg);
        if (pll1 == ms5351m_PLL_B) usedPllB = 1; else usedPllA = 1;
        ms5351m_WriteClkControl(1, pll1, sd1, isInt1);
        ms5351m_write(ms5351m_REGISTER_166_CLK1_PHASE, 0);
    } else {
        ms5351m_write(ms5351m_REGISTER_17_CLK1_CONTROL, 0x80);
    }

    /* CLK2 -> DIV2（PLL 见分配；与 CLK1 共享 PLL 时用同一 VCO 独立分频） */
    if (en2) {
        ms5351mPLLConfig_t pll2cfg;
        int32_t P1=0, P2=0, P3=1; uint8_t divBy4=0, rdiv2=0, isInt2=0;
        if (en1 && (pll2 == pll1)) {
            /* 与 CLK1 共享同一 PLL：用 CLK1 的 VCO 给 CLK2 独立算 DIV；
               分频比非法则强制 CLK2 与 CLK1 同频（保证一定出波） */
            if (!ms5351m_CalcDivForVCO(f2, Fvco1, &P1, &P2, &P3, &divBy4, &rdiv2, &isInt2)) {
                ms5351m_CalcDivForVCO(f1, Fvco1, &P1, &P2, &P3, &divBy4, &rdiv2, &isInt2); /* 同频分频比必然合法 */
            }
        } else {
            ms5351m_CalcDiv0(f2, &pll2cfg, &P1, &P2, &P3, &divBy4, &rdiv2, &isInt2, 0);
            ms5351m_writePLL((pll2 == ms5351m_PLL_B) ? ms5351m_REGISTER_34_PLLB_PARAMETERS
                                                   : ms5351m_REGISTER_26_PLLA_PARAMETERS, &pll2cfg);
        }
        ms5351m_WriteDivider(ms5351m_REGISTER_58_DIV2_PARAMETERS, P1, P2, P3, divBy4, rdiv2);
        if (pll2 == ms5351m_PLL_B) usedPllB = 1; else usedPllA = 1;
        ms5351m_WriteClkControl(2, pll2, sd2, isInt2);
        ms5351m_write(ms5351m_REGISTER_167_CLK2_PHASE, 0);
    } else {
        ms5351m_write(ms5351m_REGISTER_18_CLK2_CONTROL, 0x80);
    }

    uint8_t reg3 = (en0 ? 0 : 1) | (en1 ? 0 : 2) | (en2 ? 0 : 4);
    ms5351m_write(ms5351m_REGISTER_3_OUTPUT_ENABLE, reg3);

    /* PLL 复位（统一放最后、只复位本次用到的 PLL，避免"后配 PLL 复位先配 PLL"的干扰；
       参考 MS5352M 修改版 usedPllA/B 方案）。复位后 PLL 需重新锁定，建议调用方等待 ≥10ms 再量波。 */
    uint8_t rst = 0;
    if (usedPllA) rst |= (1 << 5);   /* PLL1_RST */
    if (usedPllB) rst |= (1 << 7);   /* PLL2_RST */
    ms5351m_write(ms5351m_REGISTER_177_PLL_RESET, rst);
}
