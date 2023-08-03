// 
// Author: Rien Matthijsse
// 
#include "RPi_Pico_TimerInterrupt.h"
#include "mos65C02.h"
#include "memory_sm0_clock.pio.h"
#include "memory_sm1_address.pio.h"

#define DELAY_FACTOR_SHORT() asm volatile("nop\nnop\nnop\nnop\n");
//#define DELAY_FACTOR_LONG()  asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
#define DELAY_FACTOR_LONG()  asm volatile("nop\nnop\nnop\nnop\n");

// # of clock cycles to keep rest pin low
#define RESET_COUNT  4

// mask used for the mux address/data bus: GP0-7
constexpr uint32_t BUS_MASK = 0xFF;

//
RPI_PICO_Timer IClock1(1);

volatile uint32_t  clockCount = 0UL;
uint8_t   resetCount;
boolean   inReset = false;
uint8_t   dataDir = 2;

bool addressValid = false;
bool clockActive = false;

/// <summary>
/// 
/// </summary>
/// <param name="vDebug"></param>
inline __attribute__((always_inline))
void setDebug(boolean vDebug) {
  gpio_put(pDebug, vDebug);
}

/// <summary>
/// drive the clockpin
/// </summary>
/// <param name="enable"></param>
inline __attribute__((always_inline))
void setClock(boolean clock) {
  gpio_put(uP_CLOCK, clock);
}

/// <summary>
/// drive the reset pin
/// </summary>
/// <param name="enable"></param>
inline __attribute__((always_inline))
void setReset(boolean reset) {
  gpio_put(uP_RESET, reset);
}

/// <summary>
/// drive the enable pins
/// </summary>
/// <param name="enable"></param>
inline __attribute__((always_inline))
void setEnable(uint32_t enable) {
  gpio_put_masked(en_MASK, enable);
}

/// <summary>
/// set direction of databus mux
/// </summary>
/// <param name="direction"></param>
inline __attribute__((always_inline))
void setDirInput() {
  if (dataDir != INPUT) {
    gpio_set_dir_masked(BUS_MASK, (uint32_t)0UL);
    dataDir = INPUT;
  }
}

/// <summary>
/// set direction of databus mux
/// </summary>
/// <param name="direction"></param>
inline __attribute__((always_inline))
void setDirOutput() {
  if (dataDir != OUTPUT) {
    gpio_set_dir_masked(BUS_MASK, BUS_MASK);
    dataDir = OUTPUT;
  }
}

/// <summary>
/// read the R/W pin
/// </summary>
/// <returns></returns>
inline __attribute__((always_inline))
bool getRW() {
  return gpio_get(uP_RW);
}

/// <summary>
/// read the databus
/// </summary>
/// <returns></returns>
inline __attribute__((always_inline))
void getData() {
  // we are already in INPUT
  //setDir(INPUT);

  setEnable(en_D0_7);
  DELAY_FACTOR_SHORT();
  data = gpio_get_all() & BUS_MASK;
}

/// <summary>
/// write the databus
/// </summary>
/// <param name="data"></param>
inline __attribute__((always_inline))
void putData() {

  setDirOutput();
  setEnable(en_D0_7);
  gpio_put_masked(BUS_MASK, (uint32_t)data);
}

/// <summary>
/// 
/// </summary>
inline __attribute__((always_inline))
void tick6502()
{
  setDebug(true);

  if (clockActive) {
/*
    if (addressValid) {
        //------------------------------------------------------------------------------------
        // do RW action
        switch (getRW()) {
        case RW_WRITE:
          getData();
          writememory(); // @address = data
//          mem[address] = data;
//          Serial.printf("W %04X %02X\n", address, data);
          break;
        }
      }
*/
      setClock(CLOCK_LOW);
      setDirInput();
      // read A0-7
      setEnable(en_A0_7);

      DELAY_FACTOR_LONG();

      //------------------------------------------------------------------------------------
      setClock(CLOCK_HIGH);

      // get A0-7
      uint16_t addr = gpio_get_all() & BUS_MASK;

      // read A8-15
      setEnable(en_A8_15);
      DELAY_FACTOR_SHORT();
      addr |= (gpio_get_all() & BUS_MASK) << 8;
      address = addr;
      addressValid = true;

      // do RW action
      switch (getRW()) {
      case RW_READ:
//        readmemory(); // data = @address
        data = mem[address];
        putData();
//        Serial.printf("R %04X %02X\n", address, data);
        break;

      case RW_WRITE:
        getData();
        writememory(); // @address = data
        //          mem[address] = data;
        //          Serial.printf("W %04X %02X\n", address, data);
        break;

      }

    // reset mgmt
    if (inReset) {
      if (resetCount-- == 0) {
        // end of reset
        setReset(RESET_HIGH);
        inReset = false;
        //        Serial.printf("RESET release\n");
      }
    }

    clockCount++;
  } // clockActive

  setDebug(false);
}

//bool __not_in_flash_func(ClockHandler)(struct repeating_timer* t)
bool ClockHandler(struct repeating_timer* t) {
  (void)t;

  tick6502();
  return true;
}

void start_clock_program(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    clock_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);


    Serial.printf("Generating clock on pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void start_address_program(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    address_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
}

/// <summary>
/// initialise the 65C02
/// </summary>
void init6502() {
  // CLOCK
  //pinMode(uP_CLOCK, OUTPUT);
  //setClock(CLOCK_HIGH);

  // DEBUG
  pinMode(pDebug, OUTPUT);

  // RESET
  pinMode(uP_RESET, OUTPUT);
  setReset(RESET_LOW);

  // pinMode(uP_CLOCK, OUTPUT);
  // gpio_put(uP_CLOCK, false);

  // pinMode(8, OUTPUT);
  // gpio_put(8, true);

  // pinMode(9, OUTPUT);
  // gpio_put(9, false);
  
  pinMode(10, OUTPUT);
  gpio_put(10, false);
  
  for(int i = 0; i < 7; i++)
  {
    pinMode(i, OUTPUT);
    gpio_put(i, false);
  }

  // RW
  //pinMode(uP_RW, INPUT_PULLUP);

  // BUS ENABLE
  //gpio_init_mask(en_MASK); 
  //gpio_set_dir_out_masked(en_MASK); // enable as output
  //setEnable(en_NONE); // all high

  // ADDRESS
  // DATA
  //gpio_init_mask(BUS_MASK);
  //setDirInput();

  //set up clock handler
  clockActive = true;
  addressValid = false;

  // // Interval in microsecs
  // if (IClock1.attachInterrupt(100000, ClockHandler)) {
  //   Serial.printf("CLOCK driver installed [%d kHz]\n", 100);
  // }
  // else
  //   Serial.println(F("Can't set Clockspeed. Select another freq"));

  PIO pio = pio1;
  uint offset = pio_add_program(pio, &clock_program);
  Serial.printf("Loaded program at %d\n", offset);

  start_clock_program(pio, 0, offset, uP_CLOCK, 1);
  offset = pio_add_program(pio, &address_program);
  start_address_program(pio, 1, offset, uP_CLOCK, 1);

  reset6502();

  while(true)
  {
    if((pio->fstat & 0x200) == 0)
    {
      uint32_t address = pio->rxf[1];
      Serial.printf("Address: %#04X\n", address);
      //Serial.printf("FSTAT: %#08X\n", pio->fstat);
    }
    //Serial.printf("FSTAT: %#08X\n", pio->fstat);
  }

  // while(true)
  // {
  //     Serial.printf("Received: %c\n", program_getc(pio, 0));
  // }
}

/// <summary>
/// reset the 65C02
/// </summary>
void reset6502() {
  setReset(RESET_LOW);

  resetCount = RESET_COUNT;
  inReset = true;
}
