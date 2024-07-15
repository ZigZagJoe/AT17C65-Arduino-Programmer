/* SendStopBit.c generates an 2-wire Interface stop bit assumes SCL is low
stop bit is a 0 to 1 transition on SDA while SCL is high
         ____________
        /
SCL ___/
               ______
              /
SDA _________/
*/

#include "2wi.h"

extern void WriteSCL(unsigned char state);
extern void WriteSDA(unsigned char state);
extern void BitDelay(void);
extern unsigned char SetSCLHigh(void);

void SendStopBit(void)
{
	WriteSDA(0);
	BitDelay();
	SetSCLHigh();
	BitDelay();
	WriteSDA(1);
	BitDelay();
}



