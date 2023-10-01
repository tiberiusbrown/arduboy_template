#include <Arduboy2.h>

Arduboy2Base a;

void setup()
{
    a.boot();
}

void loop()
{
    while(!a.nextFrame())
        ;
    
    a.display(CLEAR_BUFFER);
}
