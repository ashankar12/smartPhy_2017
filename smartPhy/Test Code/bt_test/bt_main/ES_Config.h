#include <Arduino.h>
#include <stdint.h>

// CURRENTLY SUPPORTS UPTO 5 SERVICES!

// Includes
#include "test_service.h"
#include "events_enum.h"

// Define Events
// CURRENTLY SUPPORTS UPTO 8 EVENTS!
#define NUM_EVENTS          2

#define NO_EVENT            EVENT_NULL
#define BLE_CONNECT         EVENT_01
#define NO_EVENT            EVENT_02

// Define Initializers
#define INITIALIZER_0       Test_Init

// Define Services
#define RUN_SERVICE_0       Test_Run

// Public Functions
void run_initializers(void);
void post_event(uint8_t event_2_post);
boolean is_event_pending (void);



