/* BitDelay.c for a 3.3v device min clock pulse width low and high is 4 us generate 2us delay for bit timing
using NOP's 7.3728MHz crystal 

NB: copied verbatim but this syntax looks weird
*/
#include "2wi.h"

void BitDelay(void)
{
	char delay;
	delay = 0x03;
	do
	{
		while(--delay)
	;
	_NOP();
	return;
}

