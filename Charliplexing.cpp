/*
  Charliplexing.cpp - Using tmr0 with 1ms resolution
  
  Alex Wenger <a.wenger@gmx.de> http://arduinobuch.wordpress.com/
  Matt Mets <mahto@cibomahto.com> http://cibomahto.com/
  
  Timer init code from MsTimer2 - Javier Valencia <javiervalencia80@gmail.com>
  Misc functions from Benjamin Sonnatg <benjamin@sonntag.fr>
  
  History:
    2009-12-30 - V0.0 wrote the first version at 26C3/Berlin
    2010-01-01 - V0.1 adding misc utility functions 
      (Clear, Vertical,  Horizontal) comment are Doxygen complaints now
    2010-05-27 - V0.2 add double-buffer mode

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
 * RXduino specific:
 * Timer TMR0 used.
 * TMR0_CMIA0 used to call LoLShieldRefresh()
 * ISR Excep_TMR0_CMIA0() must be made extern in intvect.c as it is located in this file.
 * SetPortDir() and SetPortVal() map Arduino digital pin number to corresponding RX63N pin.
 * This is a simple implentation of Charlieplexing driver without brightness level support.
 */

#include <math.h>
#include "Charliplexing.h"
#include "rxduino.h"
#include "intvect.h"

#define _BV(bit) (1 << (bit))

/* -----------------------------------------------------------------  */
/** Table for the LED multiplexing cycles
 * Each frame is made of 24 bytes (for the 24 display cycles)
 * There are SHADES frames per buffer in grayscale mode (one for each brigtness)
 * and twice that many to support double-buffered grayscale.
 */
unsigned char leds[2][24];

/// Determines whether the display is in single or double buffer mode
uint8_t displayMode = SINGLE_BUFFER;

/// Flag indicating that the display page should be flipped as soon as the
/// current frame is displayed
volatile boolean videoFlipPage = false;

/// Pointer to the buffer that is currently being displayed
unsigned char* displayBuffer;

/// Pointer to the buffer that should currently be drawn to
unsigned char* workBuffer;

static bool initialized = false;

struct LEDPosition {
    uint8_t high;
    uint8_t low;
};


/* -----------------------------------------------------------------  */
/** Table for LED Position in leds[] ram table
 */

const LEDPosition ledMap[126] = {
    {13, 5}, {13, 6}, {13, 7}, {13, 8}, {13, 9}, {13,10}, {13,11}, {13,12}, {13, 4}, { 4,13}, {13, 3}, { 3,13}, {13, 2}, { 2,13},
    {12, 5}, {12, 6}, {12, 7}, {12, 8}, {12, 9}, {12,10}, {12,11}, {12,13}, {12, 4}, { 4,12}, {12, 3}, { 3,12}, {12, 2}, { 2,12},
    {11, 5}, {11, 6}, {11, 7}, {11, 8}, {11, 9}, {11,10}, {11,12}, {11,13}, {11, 4}, { 4,11}, {11, 3}, { 3,11}, {11, 2}, { 2,11},
    {10, 5}, {10, 6}, {10, 7}, {10, 8}, {10, 9}, {10,11}, {10,12}, {10,13}, {10, 4}, { 4,10}, {10, 3}, { 3,10}, {10, 2}, { 2,10},
    { 9, 5}, { 9, 6}, { 9, 7}, { 9, 8}, { 9,10}, { 9,11}, { 9,12}, { 9,13}, { 9, 4}, { 4, 9}, { 9, 3}, { 3, 9}, { 9, 2}, { 2, 9},
    { 8, 5}, { 8, 6}, { 8, 7}, { 8, 9}, { 8,10}, { 8,11}, { 8,12}, { 8,13}, { 8, 4}, { 4, 8}, { 8, 3}, { 3, 8}, { 8, 2}, { 2, 8},
    { 7, 5}, { 7, 6}, { 7, 8}, { 7, 9}, { 7,10}, { 7,11}, { 7,12}, { 7,13}, { 7, 4}, { 4, 7}, { 7, 3}, { 3, 7}, { 7, 2}, { 2, 7},
    { 6, 5}, { 6, 7}, { 6, 8}, { 6, 9}, { 6,10}, { 6,11}, { 6,12}, { 6,13}, { 6, 4}, { 4, 6}, { 6, 3}, { 3, 6}, { 6, 2}, { 2, 6},
    { 5, 6}, { 5, 7}, { 5, 8}, { 5, 9}, { 5,10}, { 5,11}, { 5,12}, { 5,13}, { 5, 4}, { 4, 5}, { 5, 3}, { 3, 5}, { 5, 2}, { 2, 5},
};

void LoLShieldRefresh(void);

/* -----------------------------------------------------------------  */
/** Constructor : Initialize the interrupt code. 
 * should be called in setup();
 */
void LedSign::Init(uint8_t mode)
{
	SYSTEM.PRCR.WORD = 0xA502;	// setting PRCR1 - writes enabled
	MSTP(TMR0) = 0;
	SYSTEM.PRCR.WORD = 0xA500;	// clearing PRCR1 - writes disabled

	TMR0.TCORA = 46;			// ~1ms interrupt rate

	TMR0.TCR.BIT.CCLR = 1;		// cleared by compare match A
	TMR0.TCR.BIT.CMIEA = 1;		// compare match A interrupt enabled
	TMR0.TCCR.BIT.CSS = 1;
	TMR0.TCCR.BIT.CKS = 5;		// counter = Pclk/1024 (48MHz/1024 = 46.785kHz)

    // Record whether we are in single or double buffer mode
    displayMode = mode;
    videoFlipPage = false;

    // Point the display buffer to the first physical buffer
    displayBuffer = leds[0];

    // If we are in single buffered mode, point the work buffer
    // at the same physical buffer as the display buffer.  Otherwise,
    // point it at the second physical buffer.
    if( displayMode & DOUBLE_BUFFER ) {
        workBuffer = leds[1];
    }
    else {
        workBuffer = displayBuffer;
    }

    // Clear the buffer and display it
    LedSign::Clear(0);
    LedSign::Flip(false);

    // Then start the display
    IEN(TMR0, CMIA0) = 1;		// enable compare match interrupt A
    IPR(TMR0, CMIA0) = 15;		// compare match interrupt A priority
    IR(TMR0, CMIA0) = 0;		// clear compare match interrupt A flag

    // If we are in double-buffer mode, wait until the display flips before we
    // return
    if (displayMode & DOUBLE_BUFFER)
    {
        while (videoFlipPage) {
            delay(1);
        }
    }

    initialized = true;
}


/* -----------------------------------------------------------------  */
/** Signal that the front and back buffers should be flipped
 * @param blocking if true : wait for flip before returning, if false :
 *                 return immediately.
 */
void LedSign::Flip(bool blocking)
{
    if (displayMode & DOUBLE_BUFFER)
    {
        // Just set the flip flag, the buffer will flip between redraws
        videoFlipPage = true;

        // If we are blocking, sit here until the page flips.
        while (blocking && videoFlipPage) {
            delay(1);
        }
    }
}


/* -----------------------------------------------------------------  */
/** Clear the screen completely
 * @param set if 1 : make all led ON, if not set or 0 : make all led OFF
 */
void LedSign::Clear(int set) {
    for(int x=0;x<14;x++)  
        for(int y=0;y<9;y++) 
            Set(x,y,set);
}


/* -----------------------------------------------------------------  */
/** Clear an horizontal line completely
 * @param y is the y coordinate of the line to clear/light [0-8]
 * @param set if 1 : make all led ON, if not set or 0 : make all led OFF
 */
void LedSign::Horizontal(int y, int set) {
    for(int x=0;x<14;x++)  
        Set(x,y,set);
}


/* -----------------------------------------------------------------  */
/** Clear a vertical line completely
 * @param x is the x coordinate of the line to clear/light [0-13]
 * @param set if 1 : make all led ON, if not set or 0 : make all led OFF
 */
void LedSign::Vertical(int x, int set) {
    for(int y=0;y<9;y++)  
        Set(x,y,set);
}


/* -----------------------------------------------------------------  */
/** Set : switch on and off the leds. All the position #for char in frameString:
 * calculations are done here, so we don't need to do in the
 * interrupt code
 */
void LedSign::Set(uint8_t x, uint8_t y, uint8_t c)
{
    uint8_t pin_high = ledMap[x+y*14].high;
    uint8_t pin_low  = ledMap[x+y*14].low;
    // pin_low is directly the address in the led array (minus 2 because the 
    // first two bytes are used for RS232 communication), but
    // as it is a two byte array we need to check pin_high also.
    // If pin_high is bigger than 8 address has to be increased by one

    if( c == 1 ) {
		workBuffer[(pin_low-2)*2 + (pin_high / 8)] |=  (1<<(pin_high & 0x07));   // ON
	}
	else {
		workBuffer[(pin_low-2)*2 + (pin_high / 8)] &= ~(1<<(pin_high & 0x07));   // OFF
	}
}

/* -----------------------------------------------------------------  */
/** SetPortDir : Sets direction of individual port pin
	pin: Arduino digtal pin number
	dir: INPUT or OUTPUT
*/
void LedSign::SetPortDir(unsigned char pin, unsigned char dir)
{
	switch (pin)
	{
		case 2:
			PORT2.PDR.BIT.B2 = dir; break;
		case 3:
			PORT2.PDR.BIT.B3 = dir; break;
		case 4:
			PORT2.PDR.BIT.B4 = dir; break;
		case 5:
			PORT2.PDR.BIT.B5 = dir; break;
		case 6:
			PORT3.PDR.BIT.B2 = dir; break;
		case 7:
			PORT3.PDR.BIT.B3 = dir; break;
		case 8:
			PORTC.PDR.BIT.B2 = dir; break;
		case 9:
			PORTC.PDR.BIT.B3 = dir; break;
		case 10:
			PORTC.PDR.BIT.B4 = dir; break;
		case 11:
			PORTC.PDR.BIT.B6 = dir; break;
		case 12:
			PORTC.PDR.BIT.B7 = dir; break;
		case 13:
			PORTC.PDR.BIT.B5 = dir; break;
		default:
			;
	}
}

/* -----------------------------------------------------------------  */
/** SetPortVal : Sets output value of individual port pin
	pin: Arduino digtal pin number
	dir: 0 or 1
*/
void LedSign::SetPortVal(unsigned char pin, unsigned char val)
{
	switch (pin)
	{
		case 2:
			PORT2.PODR.BIT.B2 = val; break;
		case 3:
			PORT2.PODR.BIT.B3 = val; break;
		case 4:
			PORT2.PODR.BIT.B4 = val; break;
		case 5:
			PORT2.PODR.BIT.B5 = val; break;
		case 6:
			PORT3.PODR.BIT.B2 = val; break;
		case 7:
			PORT3.PODR.BIT.B3 = val; break;
		case 8:
			PORTC.PODR.BIT.B2 = val; break;
		case 9:
			PORTC.PODR.BIT.B3 = val; break;
		case 10:
			PORTC.PODR.BIT.B4 = val; break;
		case 11:
			PORTC.PODR.BIT.B6 = val; break;
		case 12:
			PORTC.PODR.BIT.B7 = val; break;
		case 13:
			PORTC.PODR.BIT.B5 = val; break;
		default:
			;
	}
}

/* -----------------------------------------------------------------  */
/** TMR0 CMIA ISR
 */
void Excep_TMR0_CMIA0(void)
{
	LoLShieldRefresh();
}

/*
 * LoLShieldRefresh
*/
void LoLShieldRefresh(void)
{
	// 24 Cycles of Matrix
	static uint8_t cycle = 0;

	static uint8_t i = 0;

	unsigned char m;

	unsigned char get_ddrd=0, get_portd=0, get_ddrb=0, get_portb=0;
	unsigned char dd, pd, db, pb;

	if (cycle < 6) {
		get_ddrd  = _BV(cycle+2) | displayBuffer[cycle*2];
		get_portd =            displayBuffer[cycle*2];

		get_ddrb  =            displayBuffer[cycle*2+1];
		get_portb =            displayBuffer[cycle*2+1];
	} else if (cycle < 12) {
		get_ddrd =             displayBuffer[cycle*2];
		get_portd =            displayBuffer[cycle*2];

		get_ddrb  = _BV(cycle-6) | displayBuffer[cycle*2+1];
		get_portb =            displayBuffer[cycle*2+1];
	} else if (cycle < 18) {
		get_ddrd  = _BV(cycle+2-12) | displayBuffer[cycle*2];
		get_portd =            displayBuffer[cycle*2];

		get_ddrb  =            displayBuffer[cycle*2+1];
		get_portb =            displayBuffer[cycle*2+1];
	} else {
		get_ddrd =             displayBuffer[cycle*2];
		get_portd =            displayBuffer[cycle*2];

		get_ddrb  = _BV(cycle-6-12) | displayBuffer[cycle*2+1];
		get_portb =            displayBuffer[cycle*2+1];
	}

	/* ------ D ---------- */
	for(m=2; m<8; m++)
	{
	  dd = ((get_ddrd&(1<<m))>>m);
	  pd = ((get_portd&(1<<m))>>m);

	  if(dd == 0)
	  {
		  if(pd == 0)
		  {
			  LedSign::SetPortDir(m, INPUT);
			  LedSign::SetPortVal(m, 1);
		  }

		  if(pd == 1)
		  {
			  LedSign::SetPortDir(m, INPUT);
			  LedSign::SetPortVal(m, 0);
		  }
	  }

	  if(dd == 1)
	  {
		  LedSign::SetPortDir(m, OUTPUT);
		  LedSign::SetPortVal(m, pd);
	  }
	}
   /* ------- D --------- */

   /* ------- B --------- */
	for(m=8; m<=13; m++)
	{
		db = ( get_ddrb & (1<<(m-8)))>>(m-8);
		pb = (get_portb & (1<<(m-8)))>>(m-8);

		if(db == 0)
		{
		  if(pb == 0)
		  {
			  LedSign::SetPortDir(m, INPUT);
			  LedSign::SetPortVal(m, 1);
		  }
		  if(pb == 1)
		  {
			  LedSign::SetPortDir(m, INPUT);
			  LedSign::SetPortVal(m, 0);
		  }
		}

	  if(db == 1)
	  {
		  LedSign::SetPortDir(m, OUTPUT);
		  LedSign::SetPortVal(m, pb);
	  }
	}
	/* ------- B -------- */

	cycle++;
	if (cycle > 24)
	{
		cycle = 0;

		// If the page should be flipped, do it here.
		if (videoFlipPage && (displayMode & DOUBLE_BUFFER))
		{
			// TODO: is this an atomic operation?
			videoFlipPage = false;

			unsigned char* temp = displayBuffer;
			displayBuffer = workBuffer;
			workBuffer = temp;
		}
	}
}
