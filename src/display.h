#pragma once
#include <Arduino_GFX_Library.h>

class Arduino_4848_Display : public Arduino_RGB_Display
{
public:
    using Arduino_RGB_Display::Arduino_RGB_Display;

    void    begin();
    void    setBacklight(bool on = false);
    void    setBrightness(uint8_t value);
    bool    isBacklightOn();

private:
    bool    blON        = false;
    uint8_t _brightness = 255;
};

extern Arduino_4848_Display *gfx;
