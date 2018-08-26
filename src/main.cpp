#include "main.h"

#ifdef VVC_F1
/**
 * 'Blink LED' task.
 */
static void led_task(void *args) {
  int delay_ms = *(int*)args;

  while (1) {
    LED_BANK->ODR ^= (1 << LED_PIN);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
  }
}
#endif

/**
 * 'Count value' task.
 */
static void count_task(void *args) {
  int delay_ms = *(int*)args;

  while (1) {
    // Count up and draw to the display.
    ++count_val;
    oled_draw_rect(68, 28, 34, 8, 0, 0);
    oled_draw_letter_i(70, 29, count_val, 1, 'S');
    // Delay for a second-ish.
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
  };
}

/**
 * 'Draw to OLED Display' task.
 */
static void oled_display_task(void *args) {
  int delay_ms = *(int*)args;

  while(1) {
    i2c_stream_bytes(I2C1, oled_fb, OLED_FB_SIZE);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
  };
}

/**
 * Main program.
 */
int main(void) {
  // Initial clock setup.
  setup_clocks();

  #ifdef VVC_F1
    // Enable the GPIOB clock.
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    // Enable the I2C1 clock.
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // Enable the on-board LED pin. TODO: make a method.
    #if LED_PIN > 7
      LED_BANK->CRH   &= ~(0xF << ((LED_PIN-8)*4));
      // 2MHz push-pull output.
      LED_BANK->CRH   |=  (0x2 << ((LED_PIN-8)*4));
    #else
      LED_BANK->CRL   &= ~(0xF << (LED_PIN*4));
      // 2MHz push-pull output.
      LED_BANK->CRL   |=  (0x2 << (LED_PIN*4));
    #endif
    // Turn off to start.
    // (The LED on these boards turns off with 1, on with 0)
    LED_BANK->ODR     |=  (1 << LED_PIN);
    // Enable the I2C1 pins, B6/7.
    GPIOB->CRL        &= ~(0xF << (GPIO_CRL_MODE6_Pos) |
                           0xF << (GPIO_CRL_MODE7_Pos));
    GPIOB->CRL        |=  (0xE << (GPIO_CRL_MODE6_Pos) |
                           0xE << (GPIO_CRL_MODE7_Pos));
  #elif VVC_F3
    // Enable the GPIOA clock.
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // Enable the GPIOB clock.
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    // Enable the I2C1 clock.
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Set the LED pin to push-pull output, 2MHz.
    LED_BANK->MODER   &= ~(3 << (LED_PIN*2));
    LED_BANK->MODER   |=  (1 << (LED_PIN*2));
    LED_BANK->OTYPER  &= ~(1 << LED_PIN);
    LED_BANK->OSPEEDR &= ~(3 << (LED_PIN*2));
    LED_BANK->PUPDR   &= ~(1 << LED_PIN);
    LED_BANK->ODR     |=  (1 << LED_PIN);
    // Set GPIO pins B6/B7 as alternate function mode 4 for I2C1.
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL6 |
                       GPIO_AFRL_AFRL7);
    GPIOB->AFR[0] |=  ((4 << GPIO_AFRL_AFRL6_Pos) |
                       (4 << GPIO_AFRL_AFRL7_Pos));
    // Setup GPIO pins B6, B7 as open-drain alt. func. w/pullup.
    GPIOB->MODER   &= ~(GPIO_MODER_MODER6 |
                        GPIO_MODER_MODER7);
    GPIOB->MODER   |=  ((2 << GPIO_MODER_MODER6_Pos) |
                        (2 << GPIO_MODER_MODER7_Pos));
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 |
                        GPIO_OSPEEDER_OSPEEDR7);
    GPIOB->OTYPER  |=  (GPIO_OTYPER_OT_6 |
                        GPIO_OTYPER_OT_7);
    GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPDR6 |
                        GPIO_PUPDR_PUPDR7);
    GPIOB->PUPDR   |=  ((1 << GPIO_PUPDR_PUPDR6_Pos) |
                        (1 << GPIO_PUPDR_PUPDR7_Pos));
  #endif

  // Setup the I2C peripheral and SSD1306 display.
  // I think this timing value is 1MHz @ 48MHz clock speed,
  // but it doesn't need to be precise.
  i2c_initialize(I2C1, 0x50100103);
  i2c_set_addr(I2C1, 0x78);
  ssd1306_start_sequence(I2C1);
  // Draw an initial display image to the framebuffer.
  oled_draw_rect(0, 0, 128, 64, 0, 0);
  oled_draw_rect(0, 0, 128, 64, 4, 1);
  oled_draw_text(28, 29, "Count:\0", 1, 'S');

  #ifdef VVC_F1
    // Create a blinking LED task for the on-board LED.
    xTaskCreate(led_task, "Blink_LED", 128, (void*)&led_delay, configMAX_PRIORITIES-7, NULL);
  #endif
  // Create the OLED display tasks.
  xTaskCreate(count_task, "Count_Up", 128, (void*)&count_delay, configMAX_PRIORITIES-6, NULL);
  xTaskCreate(oled_display_task, "OLED_Display", 128, (void*)&display_delay, configMAX_PRIORITIES-5, NULL);
  // Start the scheduler.
  vTaskStartScheduler();

  // This should never be reached; the FreeRTOS scheduler
  // should be in charge of the program's execution after starting.
  while (1) {}
  return 0;
}
