/*
Copyright (c) 2007, István Szentandrási
Taken from fixed point lib from Markus
Copyright (c) 2007, Markus Trenkwalder

Portions taken from the Vicent 3D rendering library
Copyright (c) 2004, David Blythe, Hans Martin Will

All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation 
  and/or other materials provided with the distribution.

* Neither the name of the library's copyright owner nor the names of its 
  contributors may be used to endorse or promote products derived from this 
  software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "fixed_func.h"

namespace fixedpoint {

static const int32_t FIX16_2PI	= float2fix<16>(6.28318530717958647692f);
static const int32_t FIX16_R2PI = float2fix<16>(1.0f/6.28318530717958647692f);

static const uint16_t sin_tab[] = {
#include "fixsintab.h"
};

int32_t fixcos16(int32_t a) 
{
    int32_t v;
    /* reduce to [0,1) */
    while (a < 0) a += FIX16_2PI;
    a = fixmul<16>(a, FIX16_R2PI);
    a += 0x4000;

    /* now in the range [0, 0xffff], reduce to [0, 0xfff] */
    a >>= 4;

    v = (a & 0x400) ? sin_tab[0x3ff - (a & 0x3ff)] : sin_tab[a & 0x3ff];
	v = fixmul<16>(v, 1 << 16);
    return (a & 0x800) ? -v : v;
}

int32_t fixsin16(int32_t a)
{
    int32_t v;

    /* reduce to [0,1) */
    while (a < 0) a += FIX16_2PI;
    a = fixmul<16>(a, FIX16_R2PI);

    /* now in the range [0, 0xffff], reduce to [0, 0xfff] */
    a >>= 4;

    v = (a & 0x400) ? sin_tab[0x3ff - (a & 0x3ff)] : sin_tab[a & 0x3ff];
    v = fixmul<16>(v, 1 << 16);
    return (a & 0x800) ? -v : v;
}

int32_t fixrsqrt16(int32_t a)
{
    int32_t x;

    static const uint16_t rsq_tab[] = { /* domain 0.5 .. 1.0-1/16 */
		0xb504, 0xaaaa, 0xa1e8, 0x9a5f, 0x93cd, 0x8e00, 0x88d6, 0x8432,
    };

    int32_t i, exp;
    if (a == 0) return 0x7fffffff;
    if (a == (1<<16)) return a;

	exp = detail::CountLeadingZeros(a);
    x = rsq_tab[(a>>(28-exp))&0x7]<<1;

	exp -= 16;
    if (exp <= 0)
		x >>= -exp>>1;
    else
		x <<= (exp>>1)+(exp&1);
    if (exp&1) x = fixmul<16>(x, rsq_tab[0]);


    /* newton-raphson */
    /* x = x/2*(3-(a*x)*x) */
    i = 0;
    do {

		x = fixmul<16>((x>>1),((1<<16)*3 - fixmul<16>(fixmul<16>(a,x),x)));
    } while(++i < 3);

    return x;
}

static inline int32_t fast_div16(int32_t a, int32_t b)
{
	if ((b >> 24) && (b >> 24) + 1) {
		return fixmul<16>(a >> 8, fixinv<16>(b >> 8));
	} else {
		return fixmul<16>(a, fixinv<16>(b));
	}
}

int32_t fixsqrt16iter(int32_t a) 
{
    int32_t s;
    int32_t i;
    s = (a + (1<<16)) >> 1;
    /* 6 iterations to converge */
    for (i = 0; i < 6; i++)
		s = (s + fast_div16(a, s)) >> 1;
    return s;
}

int32_t sqrt_tbl [] =
{
 0x00000000L, 0x00010000L, 0x00016A0AL, 0x0001BB68L,
 0x00020000L, 0x00023C6FL, 0x00027312L, 0x0002A550L,
 0x0002D414L, 0x00030000L, 0x0003298BL, 0x0003510EL,
 0x000376CFL, 0x00039B05L, 0x0003BDDDL, 0x0003DF7CL,
 0x00040000L, 0x00041F84L, 0x00043E1EL, 0x00045BE1L,
 0x000478DEL, 0x00049524L, 0x0004B0BFL, 0x0004CBBCL,
 0x0004E624L, 0x00050000L, 0x00051959L, 0x00053237L,
 0x00054AA0L, 0x0005629AL, 0x00057A2BL, 0x00059159L,
 0x0005A828L, 0x0005BE9CL, 0x0005D4B9L, 0x0005EA84L,
 0x00060000L, 0x00061530L, 0x00062A17L, 0x00063EB8L,
 0x00065316L, 0x00066733L, 0x00067B12L, 0x00068EB4L,
 0x0006A21DL, 0x0006B54DL, 0x0006C847L, 0x0006DB0CL,
 0x0006ED9FL, 0x00070000L, 0x00071232L, 0x00072435L,
 0x0007360BL, 0x000747B5L, 0x00075935L, 0x00076A8CL,
 0x00077BBBL, 0x00078CC2L, 0x00079DA3L, 0x0007AE60L,
 0x0007BEF8L, 0x0007CF6DL, 0x0007DFBFL, 0x0007EFF0L,
 0x00080000L, 0x00080FF0L, 0x00081FC1L, 0x00082F73L,
 0x00083F08L, 0x00084E7FL, 0x00085DDAL, 0x00086D18L,
 0x00087C3BL, 0x00088B44L, 0x00089A32L, 0x0008A906L,
 0x0008B7C2L, 0x0008C664L, 0x0008D4EEL, 0x0008E361L,
 0x0008F1BCL, 0x00090000L, 0x00090E2EL, 0x00091C45L,
 0x00092A47L, 0x00093834L, 0x0009460CL, 0x000953CFL,
 0x0009617EL, 0x00096F19L, 0x00097CA1L, 0x00098A16L,
 0x00099777L, 0x0009A4C6L, 0x0009B203L, 0x0009BF2EL,
 0x0009CC47L, 0x0009D94FL, 0x0009E645L, 0x0009F32BL,
 0x000A0000L, 0x000A0CC5L, 0x000A1979L, 0x000A261EL,
 0x000A32B3L, 0x000A3F38L, 0x000A4BAEL, 0x000A5816L,
 0x000A646EL, 0x000A70B8L, 0x000A7CF3L, 0x000A8921L,
 0x000A9540L, 0x000AA151L, 0x000AAD55L, 0x000AB94BL,
 0x000AC534L, 0x000AD110L, 0x000ADCDFL, 0x000AE8A1L,
 0x000AF457L, 0x000B0000L, 0x000B0B9DL, 0x000B172DL,
 0x000B22B2L, 0x000B2E2BL, 0x000B3998L, 0x000B44F9L,
 0x000B504FL, 0x000B5B9AL, 0x000B66D9L, 0x000B720EL,
 0x000B7D37L, 0x000B8856L, 0x000B936AL, 0x000B9E74L,
 0x000BA973L, 0x000BB467L, 0x000BBF52L, 0x000BCA32L,
 0x000BD508L, 0x000BDFD5L, 0x000BEA98L, 0x000BF551L,
 0x000C0000L, 0x000C0AA6L, 0x000C1543L, 0x000C1FD6L,
 0x000C2A60L, 0x000C34E1L, 0x000C3F59L, 0x000C49C8L,
 0x000C542EL, 0x000C5E8CL, 0x000C68E0L, 0x000C732DL,
 0x000C7D70L, 0x000C87ACL, 0x000C91DFL, 0x000C9C0AL,
 0x000CA62CL, 0x000CB047L, 0x000CBA59L, 0x000CC464L,
 0x000CCE66L, 0x000CD861L, 0x000CE254L, 0x000CEC40L,
 0x000CF624L, 0x000D0000L, 0x000D09D5L, 0x000D13A2L,
 0x000D1D69L, 0x000D2727L, 0x000D30DFL, 0x000D3A90L,
 0x000D4439L, 0x000D4DDCL, 0x000D5777L, 0x000D610CL,
 0x000D6A9AL, 0x000D7421L, 0x000D7DA1L, 0x000D871BL,
 0x000D908EL, 0x000D99FAL, 0x000DA360L, 0x000DACBFL,
 0x000DB618L, 0x000DBF6BL, 0x000DC8B7L, 0x000DD1FEL,
 0x000DDB3DL, 0x000DE477L, 0x000DEDABL, 0x000DF6D8L,
 0x000E0000L, 0x000E0922L, 0x000E123DL, 0x000E1B53L,
 0x000E2463L, 0x000E2D6DL, 0x000E3672L, 0x000E3F70L,
 0x000E4869L, 0x000E515DL, 0x000E5A4BL, 0x000E6333L,
 0x000E6C16L, 0x000E74F3L, 0x000E7DCBL, 0x000E869DL,
 0x000E8F6BL, 0x000E9832L, 0x000EA0F5L, 0x000EA9B2L,
 0x000EB26BL, 0x000EBB1EL, 0x000EC3CBL, 0x000ECC74L,
 0x000ED518L, 0x000EDDB7L, 0x000EE650L, 0x000EEEE5L,
 0x000EF775L, 0x000F0000L, 0x000F0886L, 0x000F1107L,
 0x000F1984L, 0x000F21FCL, 0x000F2A6FL, 0x000F32DDL,
 0x000F3B47L, 0x000F43ACL, 0x000F4C0CL, 0x000F5468L,
 0x000F5CBFL, 0x000F6512L, 0x000F6D60L, 0x000F75AAL,
 0x000F7DEFL, 0x000F8630L, 0x000F8E6DL, 0x000F96A5L,
 0x000F9ED9L, 0x000FA709L, 0x000FAF34L, 0x000FB75BL,
 0x000FBF7EL, 0x000FC79DL, 0x000FCFB7L, 0x000FD7CEL,
 0x000FDFE0L, 0x000FE7EEL, 0x000FEFF8L, 0x000FF7FEL,
 0x00100000L,
};

int32_t fixsqrt16(int32_t x)
{
        /* The idea for this comes from the Alegro library, exploiting the
     * fact that,
     *            sqrt (x) = sqrt (x/d) * sqrt (d);
     *
     *            For d == 2^(n):
     *
     *            sqrt (x) = sqrt (x/2^(2n)) * 2^n
     *
     * By locating suitable n for given x such that x >> 2n is in <0,255>
     * we can use a LUT of precomputed values.
     *
     * This algorithm provides both good performance and precission;
     * on ARM this function is about 5 times faster than c-lib sqrt, whilst
     * producing errors < 1%.
     *
     * (There are faster algorithm's available; the Carmack 'magic'
     * algorithm, http://www.codemaestro.com/reviews/review00000105.html,
     * is about five times faster than this one when implemented
     * as fixed point, but it's error is much greater and grows with the
     * size of the argument (reaches about 10% around x == 800).
     *
     * Note: on systems with FPU, the clib sqrt can be noticeably faster
     *       than this function.
     */
    const int CFX_ONE = 1 << 16;
    const int CFX_255 = 255 << 16;

    int t = 0;
    int sh = 0;
    unsigned int mask = 0x40000000;
    unsigned fract = x & 0x0000ffff;
    unsigned int d1, d2;
    
    if (x <= 0)
	return 0x0000001;

    if (x > CFX_255 || x < CFX_ONE)
    {
	/*
	 * Find the highest bit set
	 */
#if __arm__
	/* This actually requires at least arm v5, but gcc does not seem
	 * to set the architecture defines correctly, and it is I think
	 * very unlikely that anyone will want to use clutter on anything
	 * less than v5.
	 */
	int bit;
	__asm__ ("clz  %0, %1\n"
		 "rsb  %0, %0, #31\n"
		 :"=r"(bit)
		 :"r" (x));

	/* make even (2n) */
	bit &= 0xfffffffe;
#else
	/* TODO -- add i386 branch using bshr */
	int bit = 30;
	while (bit >= 0)
	{
	    if (x & mask)
		break;

	    mask = (mask >> 1 | mask >> 2);
	    bit -= 2;
	}
#endif
	
	/* now bit indicates the highest bit set; there are two scenarios
	 * 
	 * 1) bit < 23:  Our number is smaller so we shift it left to maximase
	 *               precision (< 16 really, since <16,23> never goes
	 *               through here.
	 *
	 * 2) bit > 23:  our number is above the table, so we shift right
	 */

	sh = ((bit - 22) >> 1);
	if (bit >= 8)
	    t = (x >> (16 - 22 + bit));
	else
	    t = (x << (22 - 16 - bit));
    }
    else
    {
	    t = x >> 16;
    }

    /* Do a weighted average of the two nearest values */
    int32_t v1 = sqrt_tbl[t];
    int32_t v2 = sqrt_tbl[t+1];

    /*
     * 12 is fairly arbitrary -- we want integer that is not too big to cost
     * us precission
     */
    d1 = (unsigned)(fract) >> 12;
    d2 = ((unsigned)(1 << 16) >> 12) - d1;
    
    x = ((v1*d2) + (v2*d1))/(CFX_ONE >> 12);
    
    if (sh > 0)
	x = x << sh;
    else if (sh < 0)
	x = (x >> (1 + ~sh));
    
    return x;
}

} // end namespace fixedpoint
