// 
// Author: Rien Matthijsse
// 
#include "memory.h"
#include "sound.h"

#include "roms.h"

/// <summary>
/// 64k RAM
/// </summary>
uint8_t mem[MEMORY_SIZE];

// address and data registers
uint16_t address;
uint8_t  data;

/// <summary>
/// initialise memory
/// </summary>
void initmemory() {
  address = 0x0000UL;
  data = 0x00;

  // lets install some ROMS
  if (loadROMS()) {
    Serial.println("ROMs installed");
  }

  mem[KBD] = 0x00;
  mem[DSP] = 0x00;
}

/*
void readmemory() {
  if (0xD020 <= address && address < 0xD040) { // VDU/SOUND
    switch (address) {
    case 0XD020:
      data = 0x00;
      break;
    case 0xD021:
      data = getCursorX();
      break;
    case 0xD022:
      data = getCursorY();
      break;

    case 0xD030:
      data = SoundQueueIsEmpty();
      break;

    default:
      data = 0x00;
      break;
    }
  }
  else
    data = mem[address];
}
*/

/// <summary>
/// store a byte into memory
/// </summary>
void writememory() {
  /*
  if (0xD020 <= address && address < 0xD040) { // VDU/SOUND controller
    switch (address) {
    case 0XD020:
      setCommand(data);
      break;

    case 0xD030:      // SOUND CMD
      switch (data) {
      case 0:         // RESET
        SoundReset();
        break;
      case 1:        // PUSH
        SoundPushTheNote();
        break;
      }
      break;

    case 0xD031:     // NOTE
      SoundSetNote(data);
      break;

    case 0xD032:    // DURATION
      SoundSetDuration(data);
      break;
    }

    mem[address] = data;
  }
  */
  if ((0x8000 <= address && address <= 0xCFFF) || (0xF000 <= address && address <= 0xFFF9)) { // exclude writing ROM
    Serial.printf("access violation @[%04X]\n", address);
  }
  else
    mem[address] = data;
}
