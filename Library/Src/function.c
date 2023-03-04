/*
 * function.c
 *
 *  Created on: Feb 28, 2023
 *      Author: DELL
 */

#include "function.h"


float unpack754_32( uint32_t floatingToIntValue )
{
	 myfloat ieee754;
	 unsigned int mantissa = 0;
	 unsigned int exponent = 0 ;
	 unsigned int sign = 0;

	 sign = NTH_BIT(floatingToIntValue, 31);
	 for( int ix=0; ix<8; ix++)
	   exponent = (exponent | (NTH_BIT(floatingToIntValue, (30-ix))))<<1;
	 exponent = exponent>>1;
	 for( int ix=0; ix<23; ix++)
	   mantissa = (mantissa | (NTH_BIT(floatingToIntValue, (22-ix))))<<1;
	 mantissa = mantissa >> 1;

	 ieee754.raw.sign = sign;
	 ieee754.raw.exponent = exponent;
	 ieee754.raw.mantissa = mantissa;
	 return ieee754.f;
}
