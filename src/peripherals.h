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
class pSSD1306 {
public:
  pSSD1306();
  pSSD1306(pI2C* i2c_interface, int display_w, int display_h);

  virtual void init_display() = 0;
  virtual void stream_fb() = 0;

  int w, h;
protected:
  pI2C*  i2c;
  uint8_t status;

  void write_command_byte(uint8_t cmd);
  void write_data_byte(uint8_t dat);
  virtual volatile uint8_t* get_fb() = 0;
private:
};

/*
 * To avoid dynamic memory allocation, and since there are
 * only like 3 different resolutions, just make separate
 * subclasses for different resolutions / framebuffer sizes.
 * TODO: Resolutions other than 128x64.
 */
#define SSD1306_128x64_FB_SIZE (128*64)/8
class pSSD1306_128x64 : public pSSD1306 {
public:
  pSSD1306_128x64();
  pSSD1306_128x64(pI2C* i2c_interface);
  virtual void init_display();
  virtual void stream_fb();
protected:
  virtual volatile uint8_t* get_fb() { return framebuffer; }
private:
  volatile uint8_t framebuffer[SSD1306_128x64_FB_SIZE];
};

#endif
