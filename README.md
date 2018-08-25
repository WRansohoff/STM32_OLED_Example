# Overview

This is an example FreeRTOS application for boards using some common STM32 microcontrollers. It uses the I2C peripheral to drive an SSD1306 monochrome OLED display.

To learn how to use a display like this with an RTOS, two tasks are created. One task counts up an integer value and draws its new value to a framebuffer, while a second task performs the actual I2C communication with the OLED display.

Currently only 128x64-pixel screens with an address of 0x78 are supported, but I'm hoping to change that sooner or later.

The I2C peripherals are a bit different between the older F103 and the newer F303 chips; I think that I've worked out how to get them running consistently on both, but the timing values are based off a mixture of guesswork and examples listed in ST's reference manuals. Fortunately, the SSD1306 is very forgiving when it comes to timings.

# Boards

Currently, only the STM32F103C8 and STM32F303K8 are supported.
