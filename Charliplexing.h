/*
  Charliplexing.h - Library for controlling the charliplexed led board
  from JimmiePRodgers.com
  Created by Alex Wenger, December 30, 2009.
  Modified by Matt Mets, May 28, 2010.
  Released into the public domain.
*/

#ifndef Charliplexing_h
#define Charliplexing_h

#include "rxduino.h"
#include "iodefine_gcc63n.h"

#define SINGLE_BUFFER 0
#define DOUBLE_BUFFER 1
#define GRAYSCALE     2

#define DISPLAY_COLS 14     // Number of columns in the display
#define DISPLAY_ROWS 9      // Number of rows in the display
#define SHADES 2 			// Number of distinct shades to display, including black, i.e. OFF

namespace LedSign
{
    extern void Init(uint8_t mode = SINGLE_BUFFER);
    extern void Set(uint8_t x, uint8_t y, uint8_t c = 1);
    extern void Flip(bool blocking = false);
    extern void Clear(int set=0);
    extern void Horizontal(int y, int set=0);
    extern void Vertical(int x, int set=0);
    extern void SetPortDir(unsigned char pin, unsigned char val);
    extern void SetPortVal(unsigned char pin, unsigned char val);
};

#endif
