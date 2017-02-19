#include <Arduino.h>
#include <neopixel.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include "BluefruitConfig.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

//#define DEBUG
//#define DEBUG_BLUETOOTH
//#define DEBUG_ANGLE

#define ZTHRESH_LOW -3
#define ZTHRESH_HIGH 3
#define RADIAN_CONST 57.29
#define ANGLE_ERROR 5
#define INITIAL_CONFIG_OFFSET  27
#define FINAL_CONFIG_OFFSET -27
#define ACTUAL_ANGLE_TO_CODE_ANGLE 0.61
#define INITITAL_ANGLE_THRESH 1 
#define START_ANGLE_THRESH 5 //5 Degrees movement detection for movement
#define FACTORYRESET_ENABLE 1
#define END_GOAL_FACTOR 0.9

/*States of the system*/
typedef enum State_t{
  WaitingToPair,WaitingForSelection,WaitingForInitialConfig, WaitingForUserStart,
  MonitoringCycle, VictoryDance
}State_t;
State_t mystate;

/*End states of a workout*/
typedef enum limits_t{
  maxLimit, endLimit, semiLimit
}limits_t;

/*Result of a workout*/
typedef enum Result_t{
  failure, success, workComplete
}Result_t;

/*All static variables*/
static int vibPin = 9; //Pin for vibration motor
static float initAngle; //Will be initialized after user sets workout
static int startAngle; //Start angle set by workout
static float numCodeAnglesTillEnd ; //Number of codeAngles till the end
static int numReps; //Current number of reps remanining for the user
static int totalReps; //Total Number of reps done so far 
static bool reachedEnd; //Variable to check if user reached end angle
char repSuccess[4] = {3,1,0xff};
char repFailure[4] = {3,2,0xff};
char workoutComplete[3] = {4,0xff};

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/*Turn on vibration motor*/
static void buzzOn(void){
  digitalWrite(vibPin,HIGH);
}

/*Turn off vibration motor*/
static void buzzOff(void){
  digitalWrite(vibPin,LOW);
}

/*Indicate color to user*/
static void turnOnLED(char* color){
  colorWipeWrapper(color);
}

/*Returns current angle of leg*/
static float getAngle(sensors_event_t* event){
   float accx = (event->acceleration).x;
  float accy = (event->acceleration).y;
  float res = atan(accx/accy);
  return res*RADIAN_CONST;
}

static void printAccelerations(){
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
}

/*checks if Z angle is out of bounds*/
static bool isZOutOBounds(float accz){
  if ((accz < ZTHRESH_LOW) || (accz > ZTHRESH_HIGH)){
    return true;
  }
  return false; 
}

//Checks if angles are all within defined thresholds
//while monitoring. Also sets the maximum angle reached 
//by the leg
static bool isCorrectConfig(float currAngle, float accz){ 
  if (isZOutOBounds(accz)){
    return false; //Bad Z value
  }

  float finalAngle = INITIAL_CONFIG_OFFSET - (startAngle) * ACTUAL_ANGLE_TO_CODE_ANGLE;
  float err = currAngle - finalAngle;
  if (err < ANGLE_ERROR && err > -ANGLE_ERROR){
    return true;
  }
  return false;
}

/*Sends cycle result to the app*/
static void sendToApp(Result_t res){
  char* buf;
  if (res == success){
    buf = repSuccess;  
  }else if (res == failure){
    buf = repFailure;
  }else{
    buf = workoutComplete;
  }
  ble.print("AT+BLEUARTTX=");
  ble.println(buf);
}

/*Yello dance at the end of session*/
static void dancePlease(void){
  for(int i =0; i < 5;i++){
    turnOnLED("YELLOW");
    delay(100);
    turnOnLED("OFF");
    delay(100);
  }
}

/*Returns true if angle limit has
  been reached*/
static bool isLimitReached(float currAngle, limits_t limit){
  float diff = initAngle - currAngle;
  float limitAngles = limit == maxLimit ? numCodeAnglesTillEnd : END_GOAL_FACTOR*numCodeAnglesTillEnd;
  
  if (diff > limitAngles){
    return true;
  }
  return false;
}

/*Routine for failure after
  one workout/in-between workout*/
static void failRoutine(void){
  //Switch back to waiting, start buzzing
  mystate = WaitingForInitialConfig;
  buzzOn();    
  sendToApp(failure);
  turnOnLED("RED");
}

/*Routine for success after
  one workout*/
static void successRoutine(void){
  //back to initial state
  mystate = WaitingForUserStart;
  
  sendToApp(success);
  turnOnLED("BLUE");
  numReps--;
}

/*Read from BLE receive buffer*/
static int readBLE(void){
  ble.println("AT+BLEUARTRX");
  ble.readline();

  if(strcmp(ble.buffer,"OK") == 0){
    return -1;
  }
  int cmd_number = ble.buffer[0];
  #ifdef DEBUG_BLUETOOTH
   Serial.print("Command number: ");
   Serial.println(cmd_number);
  #endif

  if (cmd_number == 0){
    //Profile, read next 
    startAngle = ble.buffer[1];
    float endAngle = ble.buffer[2];
    numCodeAnglesTillEnd = (endAngle - startAngle)*ACTUAL_ANGLE_TO_CODE_ANGLE;
    numReps = ble.buffer[3];
  }
  return cmd_number;
}

void setup() {
  Serial.begin(9600);
  pinMode(vibPin,OUTPUT);
  neopixel_init();
  buzzOff();
  mystate = WaitingToPair;
  
  /* Initialise the sensor */
  if(!accel.begin()){
    Serial.println("Accelerometer not found!\n");
    return;
  }
  /* Disable command echo from Bluefruit */
  if(!ble.begin(VERBOSE_MODE)){
    Serial.println("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
  }
   if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      Serial.println("Couldn't factory reset");
    }
  }
  ble.echo(false);
}

void loop() {
  //Get accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);
  float currAngle = getAngle(&event);
  float accz = event.acceleration.z;

  #ifdef DEBUG_ANGLE
    Serial.print("Current angle: ");
    Serial.print(currAngle);
    Serial.print("Init angle: ");
    Serial.print(initAngle);
    Serial.print(", Num Code angles to reach: ");
    Serial.println(numCodeAnglesTillEnd);
  #endif
  
  switch(mystate){  
    case WaitingToPair:
     if (ble.isConnected()){
        #ifdef DEBUG
          Serial.println("Connected to BLE!");
        #endif
        //If paired, move
        buzzOn();
        mystate = WaitingForSelection;
        turnOnLED("RED");
      }
    break;
      
    case WaitingForSelection:
    {
      // Check for incoming characters from Bluefruit
      if (readBLE() == 1){
        mystate =  WaitingForInitialConfig;
      }
    }
    break;

    case WaitingForInitialConfig:
    {
      if(readBLE() == 2){
        //User has cancelled workout
        mystate = WaitingForSelection;
      }else{
        
        if (isCorrectConfig(currAngle,accz)){
          #ifdef DEBUG_BLUETOOTH
            Serial.println("Correct config reached!");
          #endif
          buzzOff();
          turnOnLED("BLUE");
          initAngle = currAngle;
          mystate = WaitingForUserStart;
        }
      }
    }
    break;
      
    case WaitingForUserStart:
    {
      if(readBLE() == 2){
        //User has cancelled workout
        mystate = WaitingForSelection;
        buzzOn();
        turnOnLED("RED");
      }else{
        float err = currAngle - initAngle;
        if (err < -START_ANGLE_THRESH) {
          #ifdef DEBUG
            Serial.println("user has started");
          #endif
          turnOnLED("GREEN");
          totalReps = 0;
          reachedEnd = false;
          mystate = MonitoringCycle;
        }
      }
    }     
    break;
  
    case MonitoringCycle:
    {
      if(readBLE() == 2){
        //User has cancelled workout
        mystate = WaitingForSelection;
        numCodeAnglesTillEnd = 0;
        buzzOn();
        turnOnLED("RED");
      }else if ((isZOutOBounds(accz))|| isLimitReached(currAngle,maxLimit)){
        failRoutine();
        #ifdef DEBUG_BLUETOOTH
          Serial.println("Wrong move!Go back to start!");
        #endif        
      }else if(currAngle > initAngle){
        if(reachedEnd){ 
          successRoutine();
          #ifdef DEBUG_BLUETOOTH
            Serial.println("Successful workout!");
          #endif
          reachedEnd = false;
          Serial.print("numReps: ");
          Serial.println(numReps);
        }else{ //Failure 
          failRoutine();
          #ifdef DEBUG_BLUETOOTH
            Serial.println("Didn't reach high bitch");
          #endif
        }
      }else if(isLimitReached(currAngle,endLimit)){
        reachedEnd = true;
        turnOnLED("PURPLE");
      }
      
      if (numReps == 0){
        //Completed workout, perform victory dance
        mystate = VictoryDance;
        sendToApp(workComplete);
      }
    }
    break;

    case VictoryDance:
    {
      numCodeAnglesTillEnd = 0;
      dancePlease();
      mystate = WaitingForSelection;
      turnOnLED("RED");
      buzzOn();
    }
    break;
  }
  delay(500);
}
