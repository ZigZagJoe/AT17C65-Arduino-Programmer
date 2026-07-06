#include <Arduino.h>
#include "pins.h"

/////////////////////////////////////////////////////////////////////////////////////////
/* constants */
/////////////////////////////////////////////////////////////////////////////////////////

// bring in the datafile
//#include "U12_CarreraPROM_1FD8.bin.h"

/////////////////////////////////////////////////////////////////////////////////////////
// Prototypes                                                                          //
/////////////////////////////////////////////////////////////////////////////////////////

// hardware dependent routines
void BitDelay(void);

void SetVCC(unsigned char);

// utility functions
void haltWithMsg(const char * str);
void bufPrint(unsigned char* buf,unsigned int len);
int bufCmp(unsigned char* buf1, unsigned char* buf2, unsigned int len); // simple buffer compare, -1 for no problem
 
/////////////////////////////////////////////////////////////////////////////////////////
// Main                                                                                //
/////////////////////////////////////////////////////////////////////////////////////////
 
void setup() {
	// Make sure second core is stopped
    multicore_reset_core1();
	  Serial.begin();
	
  	pinMode(PIN_CEO, INPUT);
  	pinMode(PIN_DATA, INPUT);

  	pinMode(PIN_CLK, OUTPUT);
  	pinMode(PIN_SEREN, OUTPUT);
  	pinMode(PIN_RESET, OUTPUT);
  	pinMode(PIN_CE, OUTPUT);
  
  	pinMode(PIN_VCC, OUTPUT_12MA);
	  pinMode(LED_BUILTIN, OUTPUT);
	
    // set pullup on data
    digitalWrite(PIN_DATA_PU, 1);
}

void loop(void) {

  bool blink = 0;
  while (Serial.peek() < 1) {
    Serial.println("Press enter to begin process.");
    blink = !blink;
    digitalWrite(LED_BUILTIN,blink);
    delay(1000);
  }

  digitalWrite(LED_BUILTIN,1);

  Serial.println("*** XC17XX Dumper ***");


  // power on chip
  SetVCC(1);

  digitalWrite(PIN_CE, 1); // cs = deassert
  digitalWrite(PIN_RESET, 1); // reset = deassert (default) aka OE = assert
  digitalWrite(PIN_SEREN, 1); // disable serial interface
  digitalWrite(PIN_CLK, 0); 
  BitDelay();

  digitalWrite(PIN_CE,0);
  digitalWrite(PIN_RESET,0);
  BitDelay();

  int i = 0;
  while (digitalRead(PIN_CEO)) {

    Serial.print(digitalRead(PIN_DATA) ? '1' : '0');

    digitalWrite(PIN_CLK,1);
    BitDelay();
    digitalWrite(PIN_CLK,0);
    BitDelay();
    
    i++;
  }

  Serial.print("\r\nDone. Read bits = ");
  Serial.println(i);

  SetVCC(0);

  // done!!! 
  digitalWrite(LED_BUILTIN,0);
  haltWithMsg("\r\nJob done.");

} /* main */

/////////////////////////////////////////////////////////////////////////////////////////
// Functions                                                                           //
/////////////////////////////////////////////////////////////////////////////////////////

// simple hex dump
void bufPrint(unsigned char* buf,unsigned int len) {
  for (unsigned int i = 0; i < len; i++) {
    Serial.printf("%02x ", buf[i]);
    if ((i % 8) == 7) 
      Serial.println();
  }
}

// simple buffer compare, -1 for no problem
int bufCmp(unsigned char* buf1, unsigned char* buf2, unsigned int len) {
  for (unsigned int i = 0; i < len; i++) 
      if (buf1[i] != buf2[i]) 
        return i;
     
  return -1; // no problem
}

// write message and spin
void haltWithMsg(const char * str) {
  SetVCC(0); // power the chip off
  Serial.println(str);
  while (1);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Hardware dependent routines                                                         //
/////////////////////////////////////////////////////////////////////////////////////////

/* BitDelay.c for a 3.3v device 
 *  min clock pulse width low and high is 4 us 
 *  generate 2us delay for bit timing
*/
inline void BitDelay(void) {
  delayMicroseconds(2);
  return;
}

inline void SetVCC(unsigned char state) {
  	digitalWrite(PIN_VCC, state);
  	delay(100); // wait for it to settle
}

//EOF
