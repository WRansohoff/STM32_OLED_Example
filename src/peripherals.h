#ifndef _VVC_PERIPHS_H
#define _VVC_PERIPHS_H

/* Forward declarations for peripheral/device classes. */
/* TODO: Maybe these should have their own files? */
class pGPIO;
class pLED;
class pI2C;
class pSSD1306;

#include "global.h"

/* Peripheral/device class declarations. */
/*
 * Base class for a GPIO pin.
 */
class pGPIO {
public:
  pGPIO();
  pGPIO(GPIO_TypeDef* pin_bank, uint8_t pin_num);
  #if    defined(VVC_F3)
    pGPIO(GPIO_TypeDef* pin_bank, uint8_t pin_num,
          uint8_t mode, uint8_t type,
          uint8_t speed, uint8_t pupdr);
  #elif  VVC_F1
    pGPIO(GPIO_TypeDef* pin_bank, uint8_t pin_num,
          uint8_t mode, uint8_t cnf);
  #endif

  // Basic GPIO control functions.
  #if defined(VVC_F3)
    void set_af(int gpio_af);
  #endif
protected:
  GPIO_TypeDef* bank;
  uint8_t       status;
  uint8_t       pin;
  #if   defined(VVC_F3)
    uint8_t     mode;
    uint8_t     type;
    uint8_t     speed;
    uint8_t     pupdr;
  #elif  VVC_F1
    uint8_t     mode;
    uint8_t     cnf;
  #endif
private:
};

/*
 * It's slightly pointless to have an 'LED' class, since
 * it's just a wrapper for a push-pull output. Whatever.
 */
class pLED : public pGPIO {
public:
  pLED();
  pLED(GPIO_TypeDef* pin_bank, int pin_num);
  void on();
  void off();
  void toggle();
protected:
private:
};

/*
 * Class representing an I2C interface.
 * TODO: Figure out timing calculations and accept an
 * interface speed in Hz or KHz.
 */
class pI2C {
public:
  pI2C();
  pI2C(I2C_TypeDef* i2c_regs,
       GPIO_TypeDef* scl_bank, int scl_pin,
       GPIO_TypeDef* sda_bank, int sda_pin
       #if defined(VVC_F3)
       , int gpio_af
       #endif
       );
  pI2C(I2C_TypeDef* i2c_regs,
       pGPIO scl_gpio, pGPIO sda_gpio);

  /*
   * I2C control methods
   */
  #if   defined(VVC_F3)
    void init(uint32_t timing_register);
    void set_addr(uint8_t addr);
    void set_num_bytes(uint8_t nbytes);
    void start();
    void stop();
  #elif VVC_F1
    void init(uint8_t freq_bits, uint8_t ccr_bits);
    // (Currently only 7-bit addresses are supported)
    void start_with_addr(uint8_t addr, uint8_t rw);
    void stop();
  #endif
  void    write_byte(uint8_t wb);
  void    write_bytes(volatile uint8_t* buf, int len);
  uint8_t read_byte();
  I2C_TypeDef* I2Cx;
protected:
  pGPIO        scl;
  pGPIO        sda;
  uint8_t      status;
private:
};

/*
 * Class for an SSD1306 OLED display.
 */
#define OLED_MAX_FB_SIZE ((128*64)/8)
class pSSD1306 {
public:
  pSSD1306();
  pSSD1306(pI2C* i2c_interface, int display_w, int display_h);

  // Display control methods.
  void init_display();
  void stream_fb();

  // Drawing methods.
  // These write to the framebuffer and don't draw to the display.
  void draw_h_line(int x, int y, int w, unsigned char color);
  void draw_v_line(int x, int y, int h, unsigned char color);
  void draw_rect(int x, int y, int w, int h,
                 int outline, unsigned char color);
  void draw_pixel(int x, int y, unsigned char color);
  void draw_letter(int x, int y, uint32_t w0, uint32_t w1,
                   unsigned char color, char size);
  void draw_letter_c(int x, int y, char c,
                     unsigned char color, char size);
  void draw_letter_i(int x, int y, int ic,
                     unsigned char color, char size);
  void draw_text(int x, int y, const char* cc,
                 unsigned char color, const char size);

  int oled_w;
  int oled_h;
protected:
  pI2C*  i2c;
  uint8_t status;
  // TODO: Better way of sizing the framebuffer.
  volatile uint8_t framebuffer[OLED_MAX_FB_SIZE];

  void write_command_byte(uint8_t cmd);
  void write_data_byte(uint8_t dat);
private:
};

#endif
