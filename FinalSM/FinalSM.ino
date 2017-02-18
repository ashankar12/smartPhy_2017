#include <Arduino.h>
#include <neopixel.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#define DEBUG

#define ZTHRESH_LOW -3
#define ZTHRESH_HIGH 3
#define RADIAN_CONST 57.29
#define ANGLE_ERROR 10
#define INITIAL_CONFIG_OFFSET  26.56 //4.5 x and 9 y 
#define FINAL_CONFIG_OFFSET 160.8// 
#define INITITAL_ANGLE_THRESH 4 //4 degrees from initial angle
#define START_ANGLE_THRESH 5 //5 Degrees movement detection for movement
#define MAX_CONTRACTION 90 //cannot do more than 90 degrees

typedef enum State_t{
  WaitingToPair,WaitingForSelection,WaitingForInitialConfig, WaitingForUserStart,
  MonitoringCycle
}State_t;
State_t mystate;

typedef enum Result_t{
  failure, success
}Result_t;

static int vibPin = 9; //Pin for vibration motor
static int initAngle; //Will be initialized after user sets workout

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

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
  //if (res < 0){
   // res = 180 - RADIAN_CONST*res;
  //}else{
  //  res = RADIAN_CONST*res;
  //}
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

  
  
  float err = currAngle - INITIAL_CONFIG_OFFSET;
  //Serial.println(err);
  if (err < ANGLE_ERROR && err > -ANGLE_ERROR){
    return true;
  }

  return false;
}

/*Sends cycle result to the app*/
static void sendToApp(Result_t res){
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
}

void loop() {
  //Get accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);
  float currAngle = getAngle(&event);
  float accz = event.acceleration.z;

  #ifdef DEBUG
    //Serial.print("Current angle: ");
    //Serial.println(currAngle);
  #endif
  
  switch(mystate){  
    case WaitingToPair:
    { //If paired, move
      buzzOn();
      mystate = WaitingForInitialConfig;
      turnOnLED("RED");
    }
    break;
      
    case WaitingForSelection:
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
        //user has started to move, monitor

        #ifdef DEBUG
          Serial.println("user has started");
        #endif
        turnOnLED("GREEN");
        mystate = MonitoringCycle;
      }
    }     
    break;
  
    case MonitoringCycle:
    {
      if ((isZOutOBounds(accz))|| (currAngle > MAX_CONTRACTION)){
        //Switch back to waiting, start buzzing
        mystate = WaitingForInitialConfig;
        buzzOn();
        sendToApp(failure);
        rainbowCycle();
        delay(500);
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
      }
    }
    break;
  }
  delay(500);
}
