/* GetByte.c reads a byte from the 2-wire Interface slave, lastbyte is used to tell slave that the read is
over */
#include "2wi.h"

extern void WriteSCL(unsigned char state);
extern void WriteSDA(unsigned char state);
extern void BitDelay(void);
extern unsigned char SetSCLHigh(void);

unsigned char GetByte(unsigned char lastbyte)
{
	/* lastbyte == 1 for last byte */
	unsigned char i, bit;
	unsigned char result = 0;
	
	DDRB &= 0xfe; /* SDA to input */
	
	for (i = 0;i < 8;i++)
	{
		/* each bit at a time, LSB first */
		SetSCLHigh();
		BitDelay();
		bit = (PINB & 0x01);
		result = (bit << (i)) | result;
		WriteSCL(0);
		BitDelay();
	}
	
	/* send ACK */
	WriteSDA(lastbyte); /* no ack on last byte ... lastbyte = 1 for the last byte */
	BitDelay();
	SetSCLHigh();
	BitDelay();
	WriteSCL(0);
	BitDelay();
	WriteSDA(1);
	BitDelay();
	
	return(result);
}