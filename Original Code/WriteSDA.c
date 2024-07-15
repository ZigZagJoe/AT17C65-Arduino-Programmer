/* WriteSDA.c */
#include "2wi.h"

void WriteSDA(unsigned char state)
{
	if (state)
		DDRB &= 0xfe; /* input ... pullup will pull high or slave will drive low */
	else
		DDRB |= 0x01; /* output ... port latch will drive low */
}