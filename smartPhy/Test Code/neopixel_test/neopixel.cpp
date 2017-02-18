#include <Adafruit_NeoPixel.h>

#include <TimerOne.h>

#include <SimpleTimer.h>
#include "neopixel.h"

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);

//static SimpleTimer blink_timer;
//static bool blink_enabled = false;
//static char* current_color = "OFF";
//static bool parity = false;

static uint32_t Wheel(byte WheelPos); 
//static void blink_timer_handler(void);

void neopixel_init (void)
{
  //Timer1.initialize(1000000);
  //Timer1.attachInterrupt(blink_timer_handler);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void run_blink_timer(void)
{
  //blink_timer.run();
}


void start_blink(void)
{
  //blink_enabled = true;
  //Timer1.start();
  //blink_timer.setTimeout(200, blink_timer_handler);
}

void end_blink(void)
{
  //blink_enabled = false;
}

void colorWipeWrapper(char* c)
{
  if (c == "RED")
  {
     colorWipe(strip.Color(RED)); 
     //current_color = "RED";
  }
  else if (c == "GREEN")
  {
    colorWipe(strip.Color(GREEN)); 
    //current_color = "GREEN";
  }
  else if (c == "BLUE")
  {
    colorWipe(strip.Color(BLUE)); 
    //current_color = "BLUE";
  }
  else
  {
    colorWipe(strip.Color(OFF)); 
    //current_color = "OFF";
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c) 
{
  for(uint16_t i=0; i<strip.numPixels(); i++) 
  {
    strip.setPixelColor(i, c);
    strip.show();
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(void) 
{
  uint16_t i, j;
  for(j=0; j<256*5; j++) 
  { 
    // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) 
    {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
  strip.show();
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
static uint32_t Wheel(byte WheelPos) 
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) 
  {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else if(WheelPos < 170) 
  {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } 
  else 
  {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

/*
static void blink_timer_handler(void)
{
  if (blink_enabled)
  {
    if (parity)
    {
      colorWipeWrapper(current_color);
    }
    else
    {
      colorWipeWrapper("OFF");
    }
    parity ^= 1;
    //blink_timer.setTimeout(200, blink_timer_handler);
    //Timer1.start();
  }
}
*/
