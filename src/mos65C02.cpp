// 
// Author: Rien Matthijsse
// 
#include "hardware/dma.h"
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

int dma_chan;

void dma_handler() {
  //Serial.printf(__FUNCTION__);
  // Clear the interrupt request.
  dma_hw->ints1 = 1u << dma_chan;
  union u32
  {
    uint32_t value;
    struct {
      uint16_t address;
      uint8_t data;
      uint8_t flags;
    } data;
  } value;

  if((pio1->fstat & 0x200) != 0)
  {
    dma_channel_abort(dma_chan);
    return;
     //Serial.print("WTF");
  }


  value.value = pio1->rxf[1];
  //uint16_t address  = (uint16_t) (( value >> 16) & 0xFFFFUL);
  bool write = value.data.flags == 0x3;
  if(write)
  {
    *(mem + value.data.address) = value.data.data;
  }

  void* address = mem + value.data.address;
  dma_channel_set_read_addr(dma_chan, address, true);
  Serial.printf("Value: %08X Address: %04X Data: %02X Type: %s Send Data: %02X\n", value.value, value.data.address, value.data.data, write ? "W" : "R" , data);
}

/// <summary>
/// initialise the 65C02
/// </summary>
void init6502() {

  // RESET
  pinMode(uP_RESET, OUTPUT);

  // RW
  pinMode(uP_RW, INPUT_PULLUP);

  // PIO
  PIO pio = pio1;
  uint offset = 0;

  offset = pio_add_program(pio, &address_program);
  Serial.printf("Loaded program at %d\n", offset);
  start_address_program(pio, 1, offset, uP_CLOCK, 1);
  
  offset = pio_add_program(pio, &clock_program);
  Serial.printf("Loaded program at %d\n", offset);
  start_clock_program(pio, 0, offset, uP_CLOCK, 10);

  // DMA
  dma_chan = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, false);
  channel_config_set_dreq(&c, pio_get_dreq(pio1, 1, true));

  dma_channel_configure(
    dma_chan,
    &c,
    &pio1_hw->txf[1], // Write address (only need to set this once)
    NULL, // Don't provide a read address yet
    1, // Write the same value many times, then halt and interrupt
    true // Don't start yet
  );

  // Tell the DMA to raise IRQ line 0 when the channel finishes a block
  dma_channel_set_irq1_enabled(dma_chan, true);

  // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
  irq_set_exclusive_handler(DMA_IRQ_1, dma_handler);
  irq_set_enabled(DMA_IRQ_1, true);

  // Manually call the handler once, to trigger the first transfer
  //dma_handler();
}

/// <summary>
/// reset the 65C02
/// </summary>
void reset6502() {
  setReset(RESET_LOW);

  resetCount = RESET_COUNT;
  inReset = true;
}
