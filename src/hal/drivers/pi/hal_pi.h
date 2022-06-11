/********************************************************************
* Description:  hal_pi.h
*               Header for Raspberry Pi GPIO drivers
*
* Author: Brian Moyer
* License: GPL Version 2
* Copyright (c) 2022.
*
* some code  taken from the bcm2835 library by::
*
* Author: Mike McCauley (mikem@open.com.au)
* Copyright (C) 2011 Mike McCauley
* see http://www.open.com.au/mikem/bcm2835/
* Copyright (c) 2012 Ben Croston - cpuinfo.*
*
* made work for Raspberry2 9/2015 Michael Haberler
* Last change: Modify for Pi5 10/2019 andypugh
* Last change: Modify for Pi400 3/2022 elovalvo
s********************************************************************/

#include "bcm2835.h"
#include "cpuinfo.h"

#define BCM2708_PERI_BASE   0x20000000
#define BCM2708_GPIO_BASE   (BCM2708_PERI_BASE + 0x200000)
#define BCM2709_PERI_BASE   0x3F000000
#define BCM2709_GPIO_BASE   (BCM2709_PERI_BASE + 0x200000)

// http://elinux.org/index.php?title=RPi_Low-level_peripherals&printable=yes
// Rev 1 Raspberry:
static unsigned char rev1_gpios[] = {0, 1, 4, 7,   8,  9, 10, 11, 14, 15, 17, 18, 21, 22, 23, 24, 25};
static unsigned char rev1_pins[] = {3, 5, 7, 26, 24, 21, 19, 23,  8, 10, 11, 12, 13, 15, 16, 18, 22};

// Rev2 Raspberry:
static unsigned char rev2_gpios[] = {2, 3, 4,  7,  8,  9, 10, 11, 14, 15, 17, 18, 22, 23, 24, 25, 27};
static unsigned char rev2_pins[] = {3, 5, 7, 26, 24, 21, 19, 23, 8,  10, 11, 12, 15, 16, 18, 22, 13};

// Raspberry2/3:
static unsigned char rpi2_gpios[] = {2, 3, 4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 };
static unsigned char rpi2_pins[] =  {3, 5, 7, 29, 31, 26, 24, 21, 19, 23, 32, 33,  8, 10, 36, 11, 12, 35, 38, 40, 15, 16, 18, 22, 37, 13 };

static __inline__ uint32_t bcm2835_peri_read(volatile uint32_t* paddr)
{
  // Make sure we dont return the _last_ read which might get lost
  // if subsequent code changes to a different peripheral
  volatile uint32_t ret = *paddr;
  return ret;
}

static __inline__ void bcm2835_peri_write(volatile uint32_t* paddr, uint32_t value)
{
  // Make sure we don't rely on the first write, which may get
  // lost if the previous access was to a different peripheral.
  *paddr = value;
  *paddr = value;
}

// Set/clear only the bits in value covered by the mask
static __inline__ void bcm2835_peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask)
{
  uint32_t v = bcm2835_peri_read(paddr);
  v = (v & ~mask) | (value & mask);
  bcm2835_peri_write(paddr, v);
}

// Function select
// pin is a BCM2835 GPIO pin number NOT RPi pin number
//      There are 6 control registers, each control the functions of a block
//      of 10 pins.
//      Each control register has 10 sets of 3 bits per GPIO pin:
//
//      000 = GPIO Pin X is an input
//      001 = GPIO Pin X is an output
//      100 = GPIO Pin X takes alternate function 0
//      101 = GPIO Pin X takes alternate function 1
//      110 = GPIO Pin X takes alternate function 2
//      111 = GPIO Pin X takes alternate function 3
//      011 = GPIO Pin X takes alternate function 4
//      010 = GPIO Pin X takes alternate function 5
//
// So the 3 bits for port X are:
//      X / 10 + ((X % 10) * 3)
void bcm2835_gpio_fsel(uint8_t pin, uint8_t mode)
{
  // Function selects are 10 pins per 32 bit word, 3 bits per pin
  volatile uint32_t* paddr = gpio + BCM2835_GPFSEL0/4 + (pin/10);
  uint8_t   shift = (pin % 10) * 3;
  uint32_t  mask = BCM2835_GPIO_FSEL_MASK << shift;
  uint32_t  value = mode << shift;
  bcm2835_peri_set_bits(paddr, value, mask);
}
