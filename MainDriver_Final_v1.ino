/**
 * ENES 100 0501 Chemical Team
 * Drop the Bason
 * 
 * Created 3/26/18
 * 
 * Creator: Kendall Price
 * Contributors: James (Servo), Tori (pH)
 */

// General Set-Up
#include "Enes100.h"
Enes100 enes("Drop the Bason", CHEMICAL, 19, 8, 9); //teamName, teamType, markerID, rxPin, txPin
float xMS, yMS, xPos, yPos, theta;
#define pi 3.14
#define tN 1.57
#define tS -1.57
#define tE 0
#define tolerance 0.01
enum states {
  LANDING_ZONE,
  NO_OBSTACLE,
  OBSTACLE,
  MISSION_SITE
};
states state = LANDING_ZONE;
/* enum obstacle {
  LEFT,
  RIGHT,
  BOTH,
  NONE
};
obstacle detected = NONE; */

// Motor Controller Set-Up
#define lm1 7
#define lm2 6
#define rm1 5
#define rm2 4

// Winch Set-Up
#include <Servo.h>
#define winchPin 10
Servo winchServo;

// pH Sensor Set-Up
#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset -0.3            //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;

// Ultrasonic Sensor Set-Up
#define trigPinL 0
#define echoPinL 1
#define trigPinR 2
#define echoPinR 3

void setup() {
  //Initialize Pins
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);
  winchServo.attach(winchPin);
  winchServo.write(90);
  pinMode(trigPinL, OUTPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(echoPinR, INPUT);

  // Get Coordinates
  updateCoordMS();
  updateCoordOSV();

  // Drive to correct y-coord & turn East
  driveYCoordMS();

  // Drive across rocky terrain
  while(xPos < 1.2){
    driveForward(1);
    while(!updateCoordOSV()){
      driveStop(0);
    }
  }

  // Readjust to correct y-coord & facing East
  driveYCoordMS();  

  // Update the state of the driver
  state = NO_OBSTACLE;
}

void loop() {
  // Wait until the OSV location has been updated
  while(!updateCoordOSV()){
    driveStop(0);
  }

  //Switch for different possible states of the OSV
  switch (state){
    case NO_OBSTACLE:
      if(xPos < xMS-.2){
        driveForward(1);
        checkForObstacles();
      } else {
        state = MISSION_SITE;
      }
      break;
    case OBSTACLE:
      //TODO - account for if only one sensor found an obstacle
      float tTurn;
      tTurn = turnToAvoidObstacle();
      driveAroundObstacle(tTurn, xPos, yPos);
      checkForObstacles();
      //TODO - what if it made it to the MS in the middle of this?
      break;
    case MISSION_SITE:
      //TODO - adjust so OSV in right location
      enes.println("Made it to the mission site!");
      missionSiteInit();
      while(true){
        missionSiteLoop();
        //TODO - adjust if unsuccessful
      }
      break;
    default:
      enes.println("*** Error - Read Impossible State ***");
  }
}

/**
 * Determines if there are any obstacles within 15 cm and
 * adjusts the state accordingly
 * return true if obstacle detected, otherwise false
 */
boolean checkForObstacles(){
  //Get the distance each sensor is detecting
  int dL = getDistance(trigPinL, echoPinL);
  int dR = getDistance(trigPinR, echoPinR);
  //Determine if there is an obstacle within 15 cm
  if(dL <= 15 || dR <= 15){
    state = OBSTACLE;
    return true;
  }
  state = NO_OBSTACLE;
  return false;
}

/**
 * Gets the distance deteced by an ultasonic sensor in cm
 * trigPin - the trigPin fot the sensor
 * echoPin - the echo pin for the sensor
 * return distance in cm
 */
int getDistance(int trigPin, int echoPin){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2;
  return distance;
}

/**
 * Determins if on top or bottom half of arena & turns accordingly
 * towards the center
 * return - theta the OSV ends up facing
 */
float turnToAvoidObstacle(){
  if(yPos > 1){
    turnToTheta(tS);
    return tS;
  } else {
    turnToTheta(tN);
    return tN;
  }
}

/**
 * N/S, E, S/N, checking for new obstacles along the way
 */
void driveAroundObstacle(float tInit, float xInit, float yInit){
  if(tInit == tN){ //facing North
    // Drive N 35 cm
    while(yPos < yInit+.35){
      driveForward(1);
      while(!updateCoordOSV()){
        driveStop(0);    
      }
    }
  } else { //facing South
    // Drive S 35 cm
    while(yPos > yInit-.35){
      driveForward(1);
      while(!updateCoordOSV()){
        driveStop(0);   
      } 
    }
  }
  // Drive E 35 cm
  turnToTheta(tE);
  while(xPos < xInit+.35){
    driveForward(1);
    while(!updateCoordOSV()){
      driveStop(0);    
    }
  }
  // Return to MS y-coord and face E
  driveYCoordMS();
}

/**
 * Lowers the winch for 5 seconds and then stops
 */
void missionSiteInit(){
  winchServo.write(130);
  delay(1500);
  winchServo.write(90);
}

/**
 * Reads and transmits the voltage & pH value
 * Code found online, slighly modified for our purpose
 */
void missionSiteLoop(){
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)  {
    pHArray[pHArrayIndex++]=analogRead(SensorPin);
    if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
    voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
    pHValue = 3.5*voltage+Offset;
    samplingTime=millis();
  }
  if(millis() - printTime > printInterval){   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
    enes.print("Voltage:");
    enes.print(voltage);
    enes.print("    pH value: ");
    enes.println(pHValue);
    printTime=millis();
  }
}

/**
 * Got this code online. Does some array averaging for calculating pH
 */
double avergearray(int* arr, int number){
  int i, max, min;
  double avg;
  long amount=0;
  if(number<=0){
    enes.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

/**
 * Drives the OSV straight North ot South to the MS
 * y-coordinate, then turns East
 */
void driveYCoordMS(){
  //if South of MS y-coord
  while(yPos<yMS){
    enes.println("Am south of mission site");
    if(abs(theta-tN) > tolerance){
      turnToTheta(tN); //face North
      enes.print("Facing north w/ theta ");
      enes.println(theta);
    }
    driveForward(1);
    while(!updateCoordOSV()){
        driveStop(0);
    }
  }
  driveStop(0);
  //if North of MS y-coord
  while(yPos>yMS){
    enes.println("Am north of mission site");
    if(abs(theta-tS) > tolerance){
      turnToTheta(tS); //face South
      enes.print("Facing south w/ theta ");
      enes.println(theta);
    }
    driveForward(1);
    while(!updateCoordOSV()){
        driveStop(0);
    }
  }
  driveStop(0);
  //at correct y-coord - now turn East
  enes.println("At the MS y-coord!");
  turnToTheta(tE);
}

/**
 * Turns the OSV until it is facing the inputed direction
 * angle = desired direction in radians
 */
void turnToTheta(float angle){
  float delta = max(theta, angle) - min(theta, angle);
  enes.print("--- Delta: ");
  enes.println(delta);
  if(delta > 2*tolerance){
    if(delta > pi && theta > angle){
      enes.println("Case 1");
      while(theta-angle > tolerance){
        driveLeft(1);
        while(!updateCoordOSV()){
          driveStop(0);
        }
      }
      driveStop(0);
    } else if(delta > pi && theta < angle){
      enes.println("Case 2");
      while(tolerance < angle-theta){
        driveRight(1);
        while(!updateCoordOSV()){
          driveStop(0);
        }
      }
      driveStop(0);
    } else if(delta < pi && theta > angle){
      enes.println("Case 3");
      while(theta-angle > tolerance){
        driveRight(1);
        while(!updateCoordOSV()){
          driveStop(0);
        }
      }
      driveStop(0);
    } else if(delta < pi && theta < angle){
      enes.println("Case 4");
      while(tolerance < angle-theta){
        driveLeft(1);
        while(!updateCoordOSV()){
          driveStop(0);
        }
      }
      driveStop(0);
    }
  } 
}

/**
 * Retrieves mission site coordinates and sets the
 * gloabal variables xMS and yMS
 */
void updateCoordMS(){
  while (!enes.retrieveDestination()) {
    enes.println(" *** Unable to retrieve mission site coordinates *** ");
  }
  xMS = enes.destination.x;
  yMS = enes.destination.y;
  enes.print("Mission site at (");
  enes.print(xMS);
  enes.print(", ");
  enes.print(yMS);
  enes.println(+ ")");
}

/**
 * Updates the global variables of the OSV's
 * xPos, yPos, and angle theta
 */
boolean updateCoordOSV(){
  if(!enes.updateLocation()){
    enes.println(" *** Unable to update location. ***");
    return false;
  }else{
    // Update current position
    theta = enes.location.theta;
    xPos = enes.location.x;
    yPos = enes.location.y;
    return true;
  }
}

/**
 * Drives forwards for a specified time
 * time = miliseconds, 0 for indefinitely
 */
void driveForward(int time){
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, HIGH);
  digitalWrite(rm1, LOW);
  digitalWrite(rm2, HIGH);
  delay(time);
}

/**
 * Drives forwards for a specified time
 * time = miliseconds, 0 for indefinitely
 */
void driveBackward(int time){
  digitalWrite(lm1, HIGH);
  digitalWrite(lm2, LOW);
  digitalWrite(rm1, HIGH);
  digitalWrite(rm2, LOW);
  delay(time);
}

/**
 * Turns right for a specified time
 * time = miliseconds, 0 for indefinitely
 */
void driveRight(int time){
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, HIGH);
  digitalWrite(rm1, HIGH);
  digitalWrite(rm2, LOW);
  delay(time);
}

/**
 * Turns left for a specified time
 * time = miliseconds, 0 for indefinitely
 */
void driveLeft(int time){
  digitalWrite(lm1, HIGH);
  digitalWrite(lm2, LOW);
  digitalWrite(rm1, LOW);
  digitalWrite(rm2, HIGH);
  delay(time);
}

/**
 * Stops the wheel motors for a specified time
 * time = miliseconds, 0 for indefinitely
 */
void driveStop(int time){
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, LOW);
  digitalWrite(rm1, LOW);
  digitalWrite(rm2, LOW);
  delay(time);
}
