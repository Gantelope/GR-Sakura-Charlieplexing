GR-Sakura-Charlieplexing
========================

Port of Charlieplexing library for GR_Sakura RXduino board using the Renesas RX63N microcontroller.

Timer TMR0 is used to generate a 1ms tick interrupt. The ISR for this timer, Excep_TMR0_CMIA0() is in the file Charlieplexing.cpp. Ensure the vector table contains the address of this ISR. The standard environment will contain the correct vector table although it may be necessary to extern the reference to Excep_TMR0_CMIA0() in INTVECT.C.

It is the intention that this library will be used with a LOL shield. This library has been tested on an Olimex LOL shield - https://www.olimex.com/Products/Duino/Shields/SHIELD-LOL/SHIELD-LOL-3MM-RED-ASM/

One of the GR-Sakura pins used by the library is connected to Vcc via a pull-up resistor. This connection results in ghosting on one row of the LOL shield. To prevent ghosting this connection must be removed. The signal in question is CN7-CON10-5 CPU_PC7_SPIMOSI. The easiest way to disconnect the pull-up is to cut the track from resistor pack RP4. As this is a resistor pack there is no guarantee of orientation so ignore the text on the pack and cut the track from the pin at the top right on RP4. When holding the board with the Ethernet connector on the top and to the left the pin on RP4 is the one closest to switch SW2.

The removal of the pull-up may cause problems if SPI devices are used in the future such as the SD card interface. In such situations a pull-up resistor to 3.3v could be added using the CON10 header.

A video of the GR-Sakura board driving a LOL shield - http://youtu.be/wZMsghY-n6c
