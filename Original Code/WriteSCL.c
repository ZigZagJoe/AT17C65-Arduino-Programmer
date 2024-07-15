/* WriteSCL.c */
#include "2wi.h"

void WriteSCL(unsigned char state)
{
	if (state)
		DDRB &= 0xfd; /* input ... pullup will pull high or slave will drive low */
	else
		DDRB |= 0x02; /* output ... port latch will drive low */
}
