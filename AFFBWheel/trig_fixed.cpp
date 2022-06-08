/*
 * Fixed point multiplication.
 *
 * Multiply two fixed point numbers in u16,16 (unsigned 0.16) format.
 * Returns result in the same format.
 * Rounds to nearest, ties rounded up.
 */

#include "trig_fixed.h"
 
static uint16_t mul_fix_u16(uint16_t x, uint16_t y)
{
    uint16_t result;
#ifdef USE_AVR_ASM
    /* Optimized ASM version. */
    asm volatile(
      "mul  %B1, %B2\n\t"
      "movw %A0, r0\n\t"
      "ldi  r19, 0x80\n\t"
      "clr  r18\n\t"
      "mul  %A1, %A2\n\t"
      "add  r19, r1\n\t"
      "adc  %A0, r18\n\t"
      "adc  %B0, r18\n\t"
      "mul  %B1, %A2\n\t"
      "add  r19, r0\n\t"
      "adc  %A0, r1\n\t"
      "adc  %B0, r18\n\t"
      "mul  %A1, %B2\n\t"
      "add  r19, r0\n\t"
      "adc  %A0, r1\n\t"
      "adc  %B0, r18\n\t"
      "clr  r1"
        : "=&r" (result)
        : "r" (x), "r" (y)
        : "r18", "r19"
    );
#else
    /* Generic C version. Compiles to inefficient 32 bit code. */
    result = ((uint32_t) x * y + 0x8000) >> 16;
#endif
    return result;
}

/*
 * Cheap and rough fixed point multiplication: multiply only the high
 * bytes of the operands, return 16 bit result.
 *
 * For some reason, the equivalent macro compiles to inefficient code.
 * This compiles to 3 instructions (mul a,b; movw res,r0; clr r1).
 */
static uint16_t mul_high_bytes(uint16_t x, uint16_t y)
{
    return (uint8_t)(x >> 8) * (uint8_t)(y >> 8);
}

/*
 * Fixed point cos() function: sixth degree polynomial approximation.
 *
 * argument is in units of 2*M_PI/2^16.
 * result is in units of 1/2^14 (range = [-2^14 : 2^14]).
 *
 * Uses the approximation
 *      cos(M_PI/2*x) ~ P(x^2), with
 *      P(u) = (1 - u) * (1 - u * (0.23352 - 0.019531 * u))
 * for x in [0 : 1]. Max error = 9.53e-5
 */
int16_t cos_fix(uint16_t x)
{
    uint16_t y, s;
    uint8_t i = (x >> 8) & 0xc0;  // quadrant information
    x = (x & 0x3fff) << 1;        // .15
    if (i & 0x40) x = FIXED(1, 15) - x;
    x = mul_fix_u16(x, x) << 1;   // .15
    y = FIXED(1, 15) - x;         // .15
    s = FIXED(0.23361, 16) - mul_high_bytes(FIXED(0.019531, 17), x); // .16
    s = FIXED(1, 15) - mul_fix_u16(x, s);  // .15
    s = mul_fix_u16(y, s);        // .14
    return (i == 0x40 || i == 0x80) ? -s : s;
}

/*
 * Fixed point atan() function: sixth degree polynomial approximation.
 *
 * argument is in units of 1/2^15 (range = [0 : 2^15]).
 * result is in units of 2*M_PI/2^16 (range = [0 : 2^13]).
 *
 * Uses the approximation
 *      atan(x) ~ M_PI/4 * P(x), with
 *      P(x) = x * (1 + (1 - x) * Q(x))
 *      Q(x) = 0.271553 + x*(0.299045 + x*(-0.270519 + x*0.0625))
 * for x in [0 : 1]. Max error = 1.31e-4
 */
uint16_t atan_fix(uint16_t x)  // x: .15
{
    uint16_t s;
    s = FIXED(0.0625, 18);                        // .18
    s = FIXED(0.270519, 17) - mul_high_bytes(x, s);    // .17
    s = FIXED(0.299045, 16) - mul_fix_u16(x, s);     // .16
    s = FIXED(0.271553, 15) + mul_fix_u16(x, s);     // .15
    s = FIXED(1, 14)+mul_fix_u16(FIXED(1, 15)-x, s); // .14
    return mul_fix_u16(x, s);                        // .13
}

/*
 * Fixed point atan2().
 *
 * arguments are in arbitrary units, as long as they have the *same*
 * unit, range is [-32767 : +32767], -32768 does *not* work!
 * result is in units of 2*M_PI/2^16 (range = [0 : 2^13]).
 *
 * Reduces the argument to the first octant and calculates atan(y/x).
 * The octants are numbered, CCW: 0, 1, 5, 4, 6, 7, 3, 2.
 */
int16_t atan2_fix(int16_t y, int16_t x)
{
    static const uint8_t axis[8] = {0x00, 0x40, 0x00, 0xc0,
        0x80, 0x40, 0x80, 0xc0};
    uint8_t octant = 0;
    if (x < 0) { x = -x; octant |= 4; }
    if (y < 0) { y = -y; octant |= 2; }
    if (y > x) { int16_t tmp = x; x = y; y = tmp; octant |= 1; }
    uint16_t angle = atan_fix((((uint32_t) y) << 15) / x);
    if ((octant ^ (octant>>1) ^ (octant>>2)) & 1) angle = -angle;
    return angle + (axis[octant] << 8);
}
