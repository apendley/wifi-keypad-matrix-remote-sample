#ifndef __CONFIG_PINS_H__
#define __CONFIG_PINS_H__

// CHANGE THESE AT YOUR OWN RISK! 
// Not all pins support all features, so they aren't all interchangeable.

// ------------------------------
// Keypad
// ------------------------------
#define R1    A1
#define R2    SCK
#define R3    TX
#define R4    SDA
#define C1    A3
#define C2    A0
#define C3    SCL

// ------------------------------
// Status LED
// ------------------------------
#define PIN_STATUS_LED           MISO
#define PIN_STATUS_LED_POWER     MOSI

// ------------------------------
// Tilt to wake
// ------------------------------
#define PIN_TILT        RX
#define GPIO_WAKE       GPIO_NUM_16

// ------------------------------
// Voltage divider from LiPo BFF
// ------------------------------
#define PIN_VOLTAGE     A2

#endif