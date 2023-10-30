#pragma once

#include <Arduboy2.h>
#ifndef RASTY_BUFFER
#define RASTY_BUFFER Arduboy2::sBuffer
#endif

namespace rasty
{

struct dvec2 { int16_t x, y; };

void tri(dvec2 v0, dvec2 v1, dvec2 v2, uint8_t pati);

int16_t interp(int16_t a, int16_t b, int16_t c, int16_t x, int16_t z);
uint16_t inv8(uint8_t x);
uint16_t inv16(uint16_t x);

}

#ifdef RASTY_IMPLEMENTATION

namespace rasty
{

constexpr uint8_t FB_FRAC_BITS = 3;
constexpr uint8_t FB_FRAC_COEF = 1 << FB_FRAC_BITS;
constexpr uint8_t FB_FRAC_MASK = FB_FRAC_COEF - 1;

[[gnu::always_inline]] static int16_t div_frac_s(int16_t x)
{
#if ARDUINO_ARCH_AVR
    // avr-gcc generates a loop for asr of 3 and up
    // below costs one extra instruction but saves 9 cycles per invocation
    asm volatile(
        "asr  %B[x]      \n"
        "ror  %A[x]      \n"
        "asr  %B[x]      \n"
        "ror  %A[x]      \n"
        "asr  %B[x]      \n"
        "ror  %A[x]      \n"
        : [x] "+r" (x)
    );
    return x;
#else
    return x >> FB_FRAC_BITS;
#endif
}

template<class T> [[gnu::always_inline]] void tswap(T& a, T& b)
{
    T t = a;
    a = b;
    b = t;
}

template<class T> [[gnu::always_inline]] T tabs(T x)
{
    return x < 0 ? -x : x;
}


template<class T> [[gnu::always_inline]] T tmin(T a, T b) { return a < b ? a : b; }
template<class T> [[gnu::always_inline]] T tmax(T a, T b) { return a < b ? b : a; }

template<class T> [[gnu::always_inline]] T tmin(T a, T b, T c) { return tmin(tmin(a, b), c); }
template<class T> [[gnu::always_inline]] T tmax(T a, T b, T c) { return tmax(tmax(a, b), c); }

static void vline(uint8_t x, int16_t y0, int16_t y1, uint8_t pati);
static void tri_segment(int16_t ax, int16_t ay0, int16_t ay1, int16_t bx, int16_t by0, int16_t by1, uint8_t pati);

__attribute__((used)) void tri(dvec2 v0, dvec2 v1, dvec2 v2, uint8_t pati)
{
    //if(tmax(v0.y, v1.y, v2.y) < 0) return;
    if(int8_t(
        uint8_t(uint16_t(v0.y) >> 8) &
        uint8_t(uint16_t(v1.y) >> 8) &
        uint8_t(uint16_t(v2.y) >> 8)) < 0)
        return;
    if(tmin(v0.y, v1.y, v2.y) > HEIGHT * FB_FRAC_COEF) return;

    // sort by x coord
    if(v0.x > v1.x) tswap(v0, v1);
    if(v1.x > v2.x) tswap(v1, v2);
    if(v0.x > v1.x) tswap(v0, v1);

    if(v2.x < 0 || v0.x >= WIDTH * FB_FRAC_COEF || v0.x == v2.x) return;

    // interpolate vt.y: between v0, v2, with vt.x = v1.x
    int16_t ty = interp(v0.x, v1.x, v2.x, v0.y, v2.y);

    // left segment
    {
        int16_t ax  = v0.x;
        int16_t bx  = v1.x;
        int16_t ay0 = v0.y;
        int16_t ay1 = v0.y;
        int16_t by0 = v1.y;
        int16_t by1 = ty;
        if(ax < 0)
        {
            ay0 = interp(ax, 0, bx, ay0, by0);
            ay1 = interp(ax, 0, bx, ay1, by1);
            ax = 0;
        }
        if(bx > WIDTH * FB_FRAC_COEF)
        {
            by0 = interp(ax, WIDTH * FB_FRAC_COEF, bx, ay0, by0);
            by1 = interp(ax, WIDTH * FB_FRAC_COEF, bx, ay1, by1);
            bx = WIDTH * FB_FRAC_COEF;
        }
        tri_segment(ax, ay0, ay1, bx, by0, by1, pati);
    }

    // right segment
    {
        int16_t ax  = v1.x;
        int16_t bx  = v2.x;
        int16_t ay0 = v1.y;
        int16_t ay1 = ty;
        int16_t by0 = v2.y;
        int16_t by1 = v2.y;
        if(ax < 0)
        {
            ay0 = interp(ax, 0, bx, ay0, by0);
            ay1 = interp(ax, 0, bx, ay1, by1);
            ax = 0;
        }
        if(bx > WIDTH * FB_FRAC_COEF)
        {
            by0 = interp(ax, WIDTH * FB_FRAC_COEF, bx, ay0, by0);
            by1 = interp(ax, WIDTH * FB_FRAC_COEF, bx, ay1, by1);
            bx = WIDTH * FB_FRAC_COEF;
        }
        tri_segment(ax, ay0, ay1, bx, by0, by1, pati);
    }
}

void tri_segment(int16_t ax, int16_t ay0, int16_t ay1, int16_t bx, int16_t by0, int16_t by1, uint8_t pati)
{
    if(ax >= bx) return;

    if(ay0 > ay1) tswap(ay0, ay1);
    if(by0 > by1) tswap(by0, by1);

    int16_t dx = bx - ax;
    int16_t dy0 = by0 - ay0;
    int16_t dy1 = by1 - ay1;
    int16_t e0, e1;
    {
        uint8_t ay0mask = uint8_t(ay0);
        uint8_t ay1mask = uint8_t(ay1);
        if(ay0 > by0) ay0mask = -ay0mask, dy0 = -dy0;
        if(ay1 > by1) ay1mask = -ay1mask, dy1 = -dy1;
        ay0mask &= FB_FRAC_MASK;
        ay1mask &= FB_FRAC_MASK;
        int8_t tax = (((uint8_t)ax & FB_FRAC_MASK) - (FB_FRAC_COEF / 2));
        e0 = div_frac_s(dx * (ay0mask - FB_FRAC_COEF) - dy0 * tax);
        e1 = div_frac_s(dx * (ay1mask - FB_FRAC_COEF) - dy1 * tax);
    }

    int8_t sy0 = (ay0 < by0 ? 1 : -1);
    int8_t sy1 = (ay1 < by1 ? 1 : -1);
    if(ay0 > by0) ay0 -= 1;
    if(ay1 > by1) ay1 -= 1;

    uint8_t pxa = (uint8_t)div_frac_s(ax);
    uint8_t pxb = (uint8_t)div_frac_s(bx + (FB_FRAC_COEF / 2));
    if(pxb >= WIDTH)
        pxb = WIDTH - 1;
    int16_t py0 = div_frac_s(ay0);
    int16_t py1 = div_frac_s(ay1);

    uint8_t t0, t1, m0, m1;
    uint8_t* p;
    uint16_t pat;
    uint8_t t;
    uint16_t z;

    __attribute__((aligned(16))) static uint8_t const MASKS[16] PROGMEM = {
        0x00, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f,
        0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe0, 0xc0, 0x80,
    };

    static uint16_t const PATTERNS[5] PROGMEM =
    {
        0x0000, 0x5500, 0x55aa, 0xffaa, 0xffff,
    };
    pat = pgm_read_word(&PATTERNS[pati]);
    if(pxa & 1)
    {
        asm volatile(R"(
                mov __tmp_reg__, %A[pat]
                mov %A[pat], %B[pat]
                mov %B[pat], __tmp_reg__
            )"
            : [pat] "+&r" (pat)
            );
    }

    asm volatile(R"(
        L%=_loop_start:
            cp   %[pxa], %[pxb]
            brne 1f
            rjmp L_%=_loop_end
        1:

            tst  %B[e0]
            brlt 3f
        2:  add  %A[py0], %[sy0]
            adc  %B[py0], __zero_reg__
            sbrc %[sy0], 7
            dec  %B[py0]
            sub  %A[e0], %A[dx]
            sbc  %B[e0], %B[dx]
            brge 2b
        3:  tst  %B[e1]
            brlt 5f
        4:  add  %A[py1], %[sy1]
            adc  %B[py1], __zero_reg__
            sbrc %[sy1], 7
            dec  %B[py1]
            sub  %A[e1], %A[dx]
            sbc  %B[e1], %B[dx]
            brge 4b
        5:
            ; if(y1 < 0 || y0 >= HEIGHT) return;
            tst  %B[py1]
            brge 6f
            rjmp L_%=_vline_end
        6:  ldi  %A[z], (%[H])
            cp   %A[py0], %A[z]
            cpc  %B[py0], __zero_reg__
            brlt 7f
            rjmp L_%=_vline_end
        7:

            ; t0 = (uint8_t)y0, t1 = (uint8_t)y1;
            mov  %[t0], %A[py0]
            mov  %[t1], %A[py1]

            ; if(y0 < 0) t0 = 0
            sbrc %B[py0], 7
            clr  %[t0]

            ; if(y1 >= HEIGHT) t1 = HEIGHT - 1
            ldi  %A[z], (%[H])
            cp   %A[py1], %A[z]
            cpc  %B[py1], __zero_reg__
            brlt 1f
            ldi  %A[z], (%[H]-1)
            mov  %[t1], %A[z]
        1:

            ; m0 and m1
            ldi  %A[z], lo8(%[M] + 8)
            ldi  %B[z], hi8(%[M] + 8)
            ldi  %A[p], 7
            mov  %[m0], %[t0]
            and  %[m0], %A[p]
            add  %A[z], %[m0]
            lpm  %[m0], %a[z]
            andi %A[z], 0xf0
            mov  %[m1], %[t1]
            and  %[m1], %A[p]
            add  %A[z], %[m1]
            lpm  %[m1], %a[z]

            ; t0 &= 0xf8;
            ; t1 &= 0xf8;
            ldi  %A[p], 0xf8
            and  %[t0], %A[p]
            and  %[t1], %A[p]

            ; p = RASTY_BUFFER + t0 * (WIDTH / 8) + x;
            ldi  %A[p], (%[W] / 8)
            mul  %[t0], %A[p]
            ldi  %A[p], lo8(%[B])
            ldi  %B[p], hi8(%[B])
            add  %A[p], r0
            adc  %B[p], r1
            clr  __zero_reg__
            add  %A[p], %[pxa]
            adc  %B[p], __zero_reg__

            ; if(t0 == t1)
            ; {
            ;     uint8_t m = m0 & m1;
            ;     uint8_t tp = *p;
            ;     tp |= (pattern & m);
            ;     tp &= (pattern | ~m);
            ;     *p = tp;
            ;     return;
            ; }
            cp   %[t0], %[t1]
            brne 1f
            and  %[m0], %[m1]
            mov  %[m1], %[m0]
            and  %[m0], %A[pat]
            com  %[m1]
            or   %[m1], %A[pat]
            ld   __tmp_reg__, %a[p]
            or   __tmp_reg__, %[m0]
            and  __tmp_reg__, %[m1]
            st   %a[p], __tmp_reg__
            rjmp L_%=_vline_end
        1:

            ; {
            ;     uint8_t m = m0;
            ;     uint8_t tp = *p;
            ;     tp |= (pattern & m);
            ;     tp &= (pattern | ~m);
            ;     *p = tp;
            ;     p += WIDTH;
            ; }
            mov  %A[z], %[m0]
            and  %[m0], %[pat]
            com  %A[z]
            or   %A[z], %[pat]
            ld   __tmp_reg__, %a[p]
            or   __tmp_reg__, %[m0]
            and  __tmp_reg__, %A[z]
            st   %a[p], __tmp_reg__
            subi %A[p], lo8(-(%[W]))
            sbci %B[p], hi8(-(%[W]))

            ; for(int8_t t = t1 - t0 - 8; t > 0; t -= 8)
            ; {
            ;     *p = pattern;
            ;     p += WIDTH;
            ; }
            sub  %[t0], %[t1]
            ;subi %[t0], -8
            ;brge L_%=_mid_end

        L_%=_mid_4:
            sbrc %[t0], 5
            rjmp L_%=_mid_2
            st   %a[p], %A[pat]
            subi %A[p], lo8(-(%[W]))
            sbci %B[p], hi8(-(%[W]))
            st   %a[p], %A[pat]
            subi %A[p], lo8(-(%[W]))
            sbci %B[p], hi8(-(%[W]))
            st   %a[p], %A[pat]
            subi %A[p], lo8(-(%[W]))
            sbci %B[p], hi8(-(%[W]))
            st   %a[p], %A[pat]
            subi %A[p], lo8(-(%[W]))
            sbci %B[p], hi8(-(%[W]))
        L_%=_mid_2:
            sbrc %[t0], 4
            rjmp L_%=_mid_1
            st   %a[p], %A[pat]
            subi %A[p], lo8(-(%[W]))
            sbci %B[p], hi8(-(%[W]))
            st   %a[p], %A[pat]
            subi %A[p], lo8(-(%[W]))
            sbci %B[p], hi8(-(%[W]))
        L_%=_mid_1:
            sbrc %[t0], 3
            rjmp L_%=_mid_end
            st   %a[p], %A[pat]
            subi %A[p], lo8(-(%[W]))
            sbci %B[p], hi8(-(%[W]))
        L_%=_mid_end:

            ; {
            ;     uint8_t m = m1;
            ;     uint8_t tp = *p;
            ;     tp |= (pattern & m);
            ;     tp &= (pattern | ~m);
            ;     *p = tp;
            ; }
            mov  %A[z], %[m1]
            and  %[m1], %[pat]
            com  %A[z]
            or   %A[z], %[pat]
            ld   __tmp_reg__, %a[p]
            or   __tmp_reg__, %[m1]
            and  __tmp_reg__, %A[z]
            st   %a[p], __tmp_reg__

        L_%=_vline_end:
            inc  %[pxa]
            add  %A[e0], %A[dy0]
            adc  %B[e0], %B[dy0]
            add  %A[e1], %A[dy1]
            adc  %B[e1], %B[dy1]

            mov __tmp_reg__, %A[pat]
            mov %A[pat], %B[pat]
            mov %B[pat], __tmp_reg__

            rjmp L%=_loop_start
        L_%=_loop_end:
        )"
        : [e0]  "+&r" (e0)
        , [e1]  "+&r" (e1)
        , [pxa] "+&r" (pxa)
        , [py0] "+&r" (py0)
        , [py1] "+&r" (py1)
        , [t0]  "=&r" (t0)
        , [t1]  "=&r" (t1)
        , [m0]  "=&r" (m0)
        , [m1]  "=&r" (m1)
        , [p]   "=&e" (p)
        , [pat] "+&r" (pat)
        , [z]   "=&z" (z)
        : [dx]  "r"   (dx)
        , [sy0] "r"   (sy0)
        , [sy1] "r"   (sy1)
        , [dy0] "r"   (dy0)
        , [dy1] "r"   (dy1)
        , [pxb] "r"   (pxb)
        , [H]   ""    (HEIGHT)
        , [M]   ""    (&MASKS[0])
        , [B]   ""    ((uint8_t*)(RASTY_BUFFER))
        , [W]   ""    (WIDTH)
        );
}

constexpr uint16_t DIVISORS[257] PROGMEM =
{
    65535, 65535, 32767, 21845, 16383, 13107, 10922,  9362,
     8191,  7281,  6553,  5957,  5461,  5041,  4681,  4369,
     4095,  3855,  3640,  3449,  3276,  3120,  2978,  2849,
     2730,  2621,  2520,  2427,  2340,  2259,  2184,  2114,
     2047,  1985,  1927,  1872,  1820,  1771,  1724,  1680,
     1638,  1598,  1560,  1524,  1489,  1456,  1424,  1394,
     1365,  1337,  1310,  1285,  1260,  1236,  1213,  1191,
     1170,  1149,  1129,  1110,  1092,  1074,  1057,  1040,
     1023,  1008,   992,   978,   963,   949,   936,   923,
      910,   897,   885,   873,   862,   851,   840,   829,
      819,   809,   799,   789,   780,   771,   762,   753,
      744,   736,   728,   720,   712,   704,   697,   689,
      682,   675,   668,   661,   655,   648,   642,   636,
      630,   624,   618,   612,   606,   601,   595,   590,
      585,   579,   574,   569,   564,   560,   555,   550,
      546,   541,   537,   532,   528,   524,   520,   516,
      511,   508,   504,   500,   496,   492,   489,   485,
      481,   478,   474,   471,   468,   464,   461,   458,
      455,   451,   448,   445,   442,   439,   436,   434,
      431,   428,   425,   422,   420,   417,   414,   412,
      409,   407,   404,   402,   399,   397,   394,   392,
      390,   387,   385,   383,   381,   378,   376,   374,
      372,   370,   368,   366,   364,   362,   360,   358,
      356,   354,   352,   350,   348,   346,   344,   343,
      341,   339,   337,   336,   334,   332,   330,   329,
      327,   326,   324,   322,   321,   319,   318,   316,
      315,   313,   312,   310,   309,   307,   306,   304,
      303,   302,   300,   299,   297,   296,   295,   293,
      292,   291,   289,   288,   287,   286,   284,   283,
      282,   281,   280,   278,   277,   276,   275,   274,
      273,   271,   270,   269,   268,   267,   266,   265,
      264,   263,   262,   261,   260,   259,   258,   257,
      256,
};

uint16_t inv8(uint8_t x)
{
    return pgm_read_word(&DIVISORS[x]);
}

static uint16_t mul_f8_u16(uint16_t a, uint8_t b)
{
#ifdef ARDUINO_ARCH_AVR
    /*
               A1 A0
                  B0
            ========
               A0*B0
            A1*B0
         ===========
            R1 R0
    */
    uint16_t r;
    asm volatile(
        "mul  %B[a], %A[b]      \n\t"
        "movw %A[r], r0         \n\t" // r = A1*B0
        "mul  %A[a], %A[b]      \n\t"
        "add  %A[r], r1         \n\t" // R0 += hi(A0*B0)
        "clr  r1                \n\t"
        "adc  %B[r], r1         \n\t" // R1 += C
        : [r] "=&r" (r)
        : [a] "r"   (a),
          [b] "r"   (b)
        :
    );
    return r;
#else
    return uint16_t((u24(a) * b) >> 8);
#endif
}

// for x >= 256: approximates 2^24 / x
// i.e., given x: 8.8, find y=1/x: 0.16
uint16_t inv16(uint16_t x)
{
    using uint24_t = __uint24;
    // initial guess
    uint16_t const* p = &DIVISORS[x >> 8];
    uint16_t y = pgm_read_word(p);
    {
        // refine initial guess by linear interpolation
        uint16_t ty = pgm_read_word(p + 1);
        uint8_t t1 = uint8_t(x);
        uint8_t t0 = 255 - t1;
        y = mul_f8_u16(y, t0) + mul_f8_u16(ty, t1) + (y >> 8);
    }
    // one iter of newton raphson to refine further
    for(uint8_t i = 0; i < 1; ++i)
    {
        uint24_t xy = uint24_t((uint32_t(y) * x) >> 8);
        // 2 - x * y
        uint24_t t = uint24_t(0x20000) - xy;
        // y' = y * (2 - x * y)
        y = uint16_t((uint32_t(t) * y) >> 16);
    }
    return y;
}

int16_t interp(int16_t a, int16_t b, int16_t c, int16_t x, int16_t z)
{
    // x + (z-x) * (b-a)/(c-a)

    if(a == c) return x;

    uint16_t xz = (x < z ? uint16_t(z - x) : uint16_t(x - z));
    uint32_t p = uint32_t(xz) * uint16_t(b - a);
    uint16_t ac = uint16_t(c - a);
    int16_t t;

    if(ac >= 256)
    {
        uint16_t i = inv16(ac);
        t = int16_t(((p >> 8) * i) >> 16);
    }
    else
    {
        uint16_t i = inv8(ac);
        t = int16_t((p * i) >> 16);
    }

    if(x > z) t = -t;
    return x + t;
}

}

#endif
