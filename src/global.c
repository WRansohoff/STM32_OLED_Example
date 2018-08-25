#include "global.h"

// Global definitions.
// OLED framebuffer.
volatile uint8_t  oled_fb[OLED_FB_SIZE];
// Value to count up and show on the display.
volatile uint16_t count_val = 0;
// Delay length in milliseconds for counting.
const    int      count_delay = 500;
const    int      display_delay = 100;
#ifdef VVC_F1
const    int      led_delay = 500;
#endif
// Core system clock speed; initial value depends on the chip.
volatile uint32_t core_clock_hz = 8000000;
