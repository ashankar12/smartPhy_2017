#include <Arduino.h>
#include <stdint.h>
#include "ES_config.h"

// Module Level Variables
static uint8_t event_frame = 0;

// Prototypes
static void run_event(void);
static void run_service(uint8_t input_event);

// Public Functions
void run_initializers(void)
{
  #ifdef INITIALIZER_0
    INITIALIZER_0();
  #endif

  #ifdef INITIALIZER_1
    INITIALIZER_1();
  #endif

  #ifdef INITIALIZER_2
    INITIALIZER_2();
  #endif

  #ifdef INITIALIZER_3
    INITIALIZER_3();
  #endif

  #ifdef INITIALIZER_4
    INITIALIZER_4();
  #endif
}

void post_event(uint8_t event_2_post)
{
  // Disable interrupts to prevent events from being missed
  noInterrupts();

  // Set event bit
  event_frame |= event_2_post;

  // Reenable interrupts
  interrupts(); 
}

boolean is_event_pending (void)
{
  if (event_frame != 0)
  {
    run_event();
    return true;
  }
  return false;
}

// Private Functions
static void run_event(void)
{
  #if (1 <= NUM_EVENTS)
    run_service(EVENT_01);
  #endif
  
  #if (2 <= NUM_EVENTS)
    run_service(EVENT_02);
  #endif
  
  #if (3 <= NUM_EVENTS)
    run_service(EVENT_03);
  #endif
  
  #if (4 <= NUM_EVENTS)
    run_service(EVENT_04);
  #endif
  
  #if (5 <= NUM_EVENTS)
    run_service(EVENT_05);
  #endif
  
  #if (6 <= NUM_EVENTS)
    run_service(EVENT_06);
  #endif
  
  #if (7 <= NUM_EVENTS)
    run_service(EVENT_07);
  #endif
  
  #if (8 <= NUM_EVENTS)
    run_service(EVENT_08);
  #endif
}

static void run_service(uint8_t input_event)
{
  boolean valid_event = false;

  noInterrupts();
  if (input_event == input_event & event_frame)
  {
    valid_event = true;
    event_frame &= ~input_event;
  }
  interrupts();
  
  if (valid_event)
  {
    #ifdef RUN_SERVICE_0
      RUN_SERVICE_0();
    #endif
  
    #ifdef RUN_SERVICE_1
      RUN_SERVICE_1();
    #endif
  
    #ifdef RUN_SERVICE_2
      RUN_SERVICE_2();
    #endif
  
    #ifdef RUN_SERVICE_3
      RUN_SERVICE_3();
    #endif
  
    #ifdef RUN_SERVICE_4
      RUN_SERVICE_4();
    #endif
  }
}


