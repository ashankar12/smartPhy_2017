#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#define DEBUG

#define ZTHRESH_LOW -4
#define ZTHRESH_HIGH 4
#define RADIAN_CONST 57.29
#define ANGLE_ERROR 10
#define INITIAL_CONFIG_OFFSET  63.1  
#define FINAL_CONFIG_OFFSET 0// 90 degrees=> 0 deg X, 10 deg Y and 0.5 deg Z
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

static double maxAngle;
static int vibPin = 9; //Pin for vibration motor

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

/*Returns current angle of leg*/
static float getAngle(sensors_event_t* event){
   float accx = (event->acceleration).x;
  float accy = (event->acceleration).y;
  return atan(accx/accy)*RADIAN_CONST;
}

static void printAccelerations(){
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
}


void setup() {
  Serial.begin(9600);
  pinMode(vibPin,OUTPUT);
  buzzOff();
  mystate = WaitingToPair;
  /* Initialise the sensor */
  if(!accel.begin()){
    Serial.println("Accelerometer not found!\n");
    return;
  }
}

//Checks if angles are all within defined thresholds
//while monitoring. Also sets the maximum angle reached 
//by the leg
static bool isCorrectConfig(){
  sensors_event_t event;
  accel.getEvent(&event);
  
  float accz = event.acceleration.z;
  if (isZOutOfBounds(accz)){
    #ifdef DEBUG
      Serial.println("Z is out of bounds!");
    #endif
    return false; //Bad Z value
  }
    
  float currAngle = getAngle(&event);
  Serial.print("Current angle: ");
  Serial.println(currAngle);
  float err = currAngle - INITIAL_CONFIG_OFFSET;
  if (err < ANGLE_ERROR && err > -ANGLE_ERROR){
    return true;
  }
  return false;
}

/*checks if Z angle is out of bounds*/
static bool isZOutOfBounds(float accz){
  if ((accz < ZTHRESH_LOW) || (accz > ZTHRESH_HIGH))
    return true;
  return false; 
}

/*Sends cycle result to the app*/
static void sendToApp(Result_t res){
}

void loop() {
  switch(mystate){
    
    case WaitingToPair:
      //If paired, move
      buzzOn();
      mystate = WaitingForInitialConfig;
      break;
      
    case WaitingForSelection:
      break;

    case WaitingForInitialConfig:
      #ifdef DEBUG
        //printAccelerations();
      #endif
      if (isCorrectConfig()){
        #ifdef DEBUG
          Serial.println("Correct config reached!");
        #endif
        buzzOff();
        //mystate = WaitingForUserStart;
      }
      break;
      
    case WaitingForUserStart:
      break;

    case MonitoringCycle:
      /* Get a new sensor event */
      sensors_event_t event;
      accel.getEvent(&event);
      float currAngle = getAngle(&event);
      float accz = event.acceleration.z;
       
      if (isZOutOfBounds(accz)|| currAngle > MAX_CONTRACTION){
        //Switch back to waiting, start buzzing
        mystate = WaitingForUserStart;
        buzzOn();
        sendToApp(failure);
      }else if(currAngle < START_ANGLE_THRESH){
        //back to initial state
        mystate = WaitingForUserStart;
        sendToApp(success);
      }
      break;
  }
}
