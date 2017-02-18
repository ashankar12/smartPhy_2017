#include <Adafruit_NeoPixel.h>
#include "neopixel.h"

void setup() 
{
  neopixel_init();
}

void loop() 
{
  // Some example procedures showing how to display to the pixels:
  //colorWipeWrapper("RED"); // Red
  //colorWipeWrapper("GREEN"); // Green
  colorWipeWrapper("BLUE"); // Blue
  //colorWipeWrapper("OFF"); // Red
  //rainbowCycle(20);
}


