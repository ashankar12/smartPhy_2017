/****************************************************

                  bt_comm_helpers.c

****************************************************/ 

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                      Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
               Module Level Variables
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                     Prototypes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                   Public Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 

bool read_bt_message (Adafruit_BluefruitLE_UART::Adafruit_BluefruitLE_UART ble)
{
  // Check for incoming characters from Bluefruit
  #if ENABLE_BT_DEBUG
    ble.println("AT+BLEUARTRX");
  #endif
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) 
  {
    // no data
    return false;
  }
  // Some data was found, its in the buffer
  #if ENABLE_BT_DEBUG
    Serial.print(F("[Recv] ")); Serial.println(ble.buffer);
  #endif

  // Make sense of currently received data
  //parse_bt_message();
  
  ble.waitForOK();

  // Read data successfully
  return true;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                   Private Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
