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
#define DEBUG_ANGLE

#define ZTHRESH_LOW -3
#define ZTHRESH_HIGH 3
#define RADIAN_CONST 57.29
#define ANGLE_ERROR 10
#define INITIAL_CONFIG_OFFSET  27
#define FINAL_CONFIG_OFFSET -27
#define ACTUAL_ANGLE_TO_CODE_ANGLE 0.61
#define INITITAL_ANGLE_THRESH 1 
#define START_ANGLE_THRESH 5 //5 Degrees movement detection for movement
#define MAX_CONTRACTION 90 //cannot do more than 90 degrees
#define BUFFER_SIZE 5
#define FACTORYRESET_ENABLE 1

/*States of the system*/
typedef enum State_t{
  WaitingToPair,WaitingForSelection,WaitingForInitialConfig, WaitingForUserStart,
  MonitoringCycle, VictoryDance
}State_t;
State_t mystate;

/*Enumerators defining axes of the 
  accelerometers. */
typedef enum axis_t{
  x,y,z
}axis_t;

typedef enum axis_type{
  major,first_minor,second_minor
}axis_type;

/*Result of a workout*/
typedef enum Result_t{
  failure, success
}Result_t;

/*All static variables*/
static int vibPin = 9; //Pin for vibration motor
static float initAngle; //Will be initialized after user sets workout
static int startAngle; //Start angle set by workout
static float numCodeAnglesTillEnd ; //Number of codeAngles till the end
static int numReps; //Current number of reps remanining for the user
static int totalReps; //Total Number of reps done so far 

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
  
  float err = currAngle - INITIAL_CONFIG_OFFSET - startAngle;
  if (err < ANGLE_ERROR && err > -ANGLE_ERROR){
    return true;
  }
  return false;
}

/*Sends cycle result to the app*/
static void sendToApp(Result_t res){
}

/*Parse BLE input and execute*/
static void parseBLEInput(){
  if(strcmp(ble.buffer,"OK") == 0){
    return;
  }
  int cmd_number = ble.buffer[0];
  #ifdef DEBUG_BLUETOOTH
   Serial.print("Command number: ");
   Serial.println(cmd_number);
  #endif
  
  switch(cmd_number){
    case 0:
      {
        //Profile, read next 
        startAngle = ble.buffer[1];
        float endAngle = ble.buffer[2];
        numCodeAnglesTillEnd = (endAngle - startAngle)*ACTUAL_ANGLE_TO_CODE_ANGLE;
        numReps = ble.buffer[3];
      }
      break;
    case 1:
      {
        //Start workout, change state
        mystate = WaitingForInitialConfig;
      }
      break;
    case 2:
      {
        //Stop workout,change state, wait for user selection
        mystate = WaitingForSelection;
      }
      break;
    default:
      break;
  }
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
static bool isLimitReached(float currAngle){
  float diff = initAngle - currAngle;
  if (diff > numCodeAnglesTillEnd){
    return true;
  }
  return false;
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
       ble.println("AT+BLEUARTRX");
       ble.readline();
       parseBLEInput();
    }
    break;

    case WaitingForInitialConfig:
    {
      #ifdef DEBUG
        //printAccelerations();
      #endif
      if (isCorrectConfig(currAngle,accz)){
        #ifdef DEBUG
          Serial.println("Correct config reached!");
        #endif
        buzzOff();
        turnOnLED("BLUE");
        initAngle = currAngle;
        mystate = WaitingForUserStart;
      }
    }
    break;
      
    case WaitingForUserStart:
    {
      float err = currAngle - initAngle;
      if (err < -START_ANGLE_THRESH) {
        #ifdef DEBUG
          Serial.println("user has started");
        #endif
        turnOnLED("GREEN");
        totalReps = 0;
        mystate = MonitoringCycle;
      }
    }     
    break;
  
    case MonitoringCycle:
    {
      if ((isZOutOBounds(accz))|| isLimitReached(currAngle)){
        //Switch back to waiting, start buzzing
        mystate = WaitingForInitialConfig;
        buzzOn();    
        sendToApp(failure);
        turnOnLED("RED");
        #ifdef DEBUG
          Serial.println("Wrong move!Go back to start!");
        #endif
      }else if(currAngle > initAngle){
        //back to initial state
        mystate = WaitingForUserStart;
        sendToApp(success);
        turnOnLED("BLUE");
        #ifdef DEBUG
          Serial.println("Successful workout!");
        #endif
        numReps--;
      }
      totalReps++;
      if (numReps == 0){
        //Completed workout, perform victory dance
        mystate = VictoryDance;
        //Send total number of reps to App
      }
    }
    break;

    case VictoryDance:
    {
      totalReps = 0;
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
