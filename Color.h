#ifndef __COLOR_H__
#define __COLOR_H__

// We're partially piggybacking off of some of Adafruit_NeoPixel's helpers.
#include <Adafruit_NeoPixel.h>

namespace Color {
    inline uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }

    inline void rgbComponents(uint32_t c, uint8_t& r, uint8_t& g, uint8_t& b) {
        r = (uint8_t)(c >> 16);
        g = (uint8_t)(c >> 8);
        b = (uint8_t)c;
    }

    inline uint8_t sine8(uint8_t x) {
        return Adafruit_NeoPixel::sine8(x);
    }

    inline uint32_t gamma32(uint32_t c) {
        return Adafruit_NeoPixel::gamma32(c);
    }

    inline uint8_t scale8(uint8_t c, uint8_t brightness) {
        return ((uint16_t)c * brightness) / 255;        
    }

    inline uint32_t scale(uint32_t color, uint8_t brightness) {
        uint8_t r = (uint8_t)(color >> 16);
        uint8_t g = (uint8_t)(color >> 8);
        uint8_t b = (uint8_t)color;

        return rgb(scale8(r, brightness), 
                   scale8(g, brightness), 
                   scale8(b, brightness));
    }
}

#endif