#include <Arduino.h>
#include "Keypad.h"
#include "Config_Pins.h"

namespace {
    const int rowCount = 4;
    const int columnCount = 3;
    const int keyCount = rowCount * columnCount;

    char keys[rowCount][columnCount] = {
        {'1', '2', '3'}, 
        {'4', '5', '6'}, 
        {'7', '8', '9'}, 
        {'*', '0', '#'}
    };

    byte rowPins[rowCount] =    {R1, R2, R3, R4};
    byte colPins[columnCount] = {C1, C2, C3};
}

Adafruit_Keypad keypad = Adafruit_Keypad(makeKeymap(keys), rowPins, colPins, rowCount, columnCount);