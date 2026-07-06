# AT17Cxx programming with Arduino / Pico 
Here is a port of Atmel's example code to program the AT17Cxx series of FPGA configuration PROMs, along with a simple PROM dump program.

AT17C65_Teensy - Original AVR-specific Arduino code. Should work on anything with PORTB/DDRB accessible and wired as noted in the file.

AT17C65_Programmer_Pico - Generic Ardunio PlatformIO port, currently configured for use with Pico2. See pins.h for wiring.

XC17XX_Dumper_Pico - Generic Ardunio PlatformIO for dumping raw bitstream. Uses same wiring as the programmer.

docs/ - Atmel-supplied documentation