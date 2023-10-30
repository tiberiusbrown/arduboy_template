#include <Arduboy2.h>

#define RASTY_IMPLEMENTATION
#include "rasty.hpp"

Arduboy2Base a;

ARDUBOY_NO_USB;

void setup()
{
    a.boot();
    //a.setFrameDuration(33);
}

static uint8_t pat = 4;

void loop()
{
    while(!a.nextFrame())
        ;

    a.pollButtons();
    if(a.justPressed(A_BUTTON))
        if(++pat > 4) pat = 0;

    for(uint16_t i = 0; i < 100; ++i)
    {
        //constexpr int F = rasty::FB_FRAC_COEF;
        constexpr int F = 1;
        constexpr int X = 50;
        constexpr int Y = 20;
        rasty::tri(
            {-25 * F + X, 0 + Y},
            {64 * F + X, 80 * F + Y},
            {128 * F + 25 * F + X, 40 * F + Y},
            pat);
    }
    
    a.display(CLEAR_BUFFER);
}
