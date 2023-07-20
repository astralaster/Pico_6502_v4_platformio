// 
// Author: Rien Matthijsse
// 

#ifndef _M6821_h
#define _M6821_h

//#include "Arduino.h"
#include <cstdint>

////////////////////////////////////////////////////////////////////
// 6821 Peripheral
// emulate just enough so keyboard/display works thru serial port.
////////////////////////////////////////////////////////////////////
//

#define KBD   0xd010
#define KBDCR 0xd011
#define DSP   0xd012
#define DSPCR 0xd013

extern uint8_t regKBD;
extern uint8_t regKBDDIR;    // Dir register when KBDCR.bit2 == 0
extern uint8_t regKBDCR;
extern uint8_t regDSP;
extern uint8_t regDSPDIR;    // Dir register when DSPCR.bit2 == 0
extern uint8_t regDSPCR; 

extern void init6821();

#endif

