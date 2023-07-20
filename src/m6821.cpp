// 
// Author: Rien Matthijsse
// 

#include "m6821.h"

uint8_t regKBD;
uint8_t regKBDDIR;    // Dir register when KBDCR.bit2 == 0
uint8_t regKBDCR;
uint8_t regDSP;
uint8_t regDSPDIR;    // Dir register when DSPCR.bit2 == 0
uint8_t regDSPCR;


/// <summary>
/// 
/// </summary>
void init6821()
{
  regKBD = 0x00;
  regKBDDIR = 0x00;
  regKBDCR = 0x00;
  regDSP = 0x00;
  regDSPDIR = 0x00;
  regDSPCR = 0x00;
}
