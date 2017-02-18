#include <Adafruit_NeoPixel.h>

#define LED_PIN 8

#define RED         255,0,0
#define GREEN       0,255,0
#define BLUE        0,0,255
#define OFF         0,0,0

void neopixel_init (void);
void colorWipe(uint32_t c);
void rainbowCycle(void);
void colorWipeWrapper(char* c);
void start_blink(void);
void end_blink(void);
void run_blink_timer(void);

