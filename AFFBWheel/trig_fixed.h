/*
Fast integer trigonometry.
Source: https://wiki.logre.eu/index.php/Trigonom%C3%A9trie_en_virgule_fixe
https://web.archive.org/web/20210506162226/https://wiki.logre.eu/index.php/Trigonom%C3%A9trie_en_virgule_fixe
*/
#pragma once

#include <stdint.h>

/*
 * cos_fix.h: Fixed-point cos() and sin() functions, based on a sixth
 * degree polynomial approximation.
 *
 * argument is in units of 2*M_PI/2^16.
 * result is in units of 1/2^14 (range = [-2^14 : 2^14]).
 *
 * The cosine function uses an even-polynomial approximation of
 * cos(M_PI/2*x) for x in [0:1], and symmetries when x is outside [0:1].
 * Sine is defined as sin(x) = cos(3*M_PI/2+x).
 */
 
/*
 * Sixth degree polynomial:
 *      cos(M_PI/2*x) ~ (1 - x^2)*(1 - x^2*(0.23352 - 0.019531*x^2))
 * for x in [0:1]. Max error = 9.53e-5
 */
int16_t cos_fix(uint16_t x);

/*
 * Fixed point sin().
 */
static inline int16_t sin_fix(uint16_t x)
{
    return cos_fix(0xc000 + x);
}

int16_t atan2_fix(int16_t y, int16_t x);


/*
 * cos_fix.c: Fixed-point cos() function.
 *
 * Compile for AVR:
 *      avr-gcc -c -mmcu=avr5 -Os -Wall -Wextra cos_fix.c
 *
 * Compile for AVR with no ASM code:
 *      avr-gcc -DNO_ASM -c -mmcu=avr5 -Os -Wall -Wextra cos_fix.c
 *
 */


#define FIXED(x, n) ((uint16_t)((float)(x) * ((uint32_t)1 << (n)) + .5))

#if !defined(NO_ASM)
# if defined(__AVR_HAVE_MUL__) && defined(__AVR_HAVE_MOVW__)
#  define USE_AVR_ASM
# endif
#endif
