/* ProgramResetPolarity.c Locations 0x20000H through 0x20003H are used to store the reset/output enable
polarity for 512K/1M.
0xff = active low reset and active low output enable.
0x00 = active high reset and active high output enable.
So the memory location values determine the reset polarity.
After programming the data polling method is used to determine the end of the internal programming cycle.
Timer/Counter1 is used to ensure the polling code exits its while loop. */
#include "2wi.h"

extern volatile unsigned char t1_timed_out;
extern unsigned char SendByte(unsigned char byte, unsigned char order);
extern void SendStartBit(void);
extern void SendStopBit(void);

void ProgramResetPolarity(unsigned char state)
{
	unsigned char i;
	unsigned char test_ack = 0xff;
	
	PORTB &= 0xf7; /* bring CS low */
	PORTB &= 0xef; /* bring RESET/OE low */
	PORTB &= 0xfb; /* bring SER_EN low */
	
	SendStartBit();
	SendByte(AT17 + WRITE,MSB_FIRST); /* send device address byte */
	SendByte(0x02,MSB_FIRST); /* 1st address byte */
	SendByte(0x00,MSB_FIRST); /* 2nd address byte */
	SendByte(0x00,MSB_FIRST); /* 3rd address byte ... most significant byte */
	
	for (i = 0; i < 4; i++)
		SendByte(state,LSB_FIRST);
		
	SendStopBit();
	t1_timed_out = FALSE; /* set in timer counter 0 overflow interrupt routine */
	
	/* 20 milli-second timeout */
	/* 7.3728MHz / 1024 = 7200 Hz */
	/* 7200 Hz = 138.8 us */
	/* 20 ms / 138.8 us = 144.09 */
	/* 65536 - 144 = 65392 = ff70 */
	/* interrupt on ffff to 0000 transition */
	
	TCNT1H = 0xff;
	TCNT1L = 0x70; /* load counter */
	TCCR1B = 0x05; /* timer/counter 1 clock / 1024 */
	
	/* continue sending start bit and device address until we get an ack back */
	/* data poll to program complete ... time out for error */
	while (test_ack && !t1_timed_out)
	{
		SendStartBit();
		test_ack = SendByte(AT17 + WRITE,MSB_FIRST); /* send device address byte */
	}
	
	SendStopBit();
	
	PORTB |= 0x04; /* bring SER_EN high */
	PORTB |= 0x10; /* bring RESET/OE high */
	PORTB |= 0x08; /* bring CS high */
}