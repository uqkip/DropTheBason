/**
 * ENES 100 0501 Chemical Team
 * Drop the Bason
 * 
 * Created 3/26/18
 * 
 * Creator: Kendall Price
 * Contributors: James (Winch), Tori (pH), Aidan (Sponge)
 */

// General Set-Up
#include "Enes100.h"
Enes100 enes("Drop the Bason", CHEMICAL, 19, 8, 9); //teamName, teamType, markerID, rxPin, txPin
float xMS, yMS, xPos, yPos, theta;
#define pi 3.14
#define tN 1.57
#define tS -1.57
#define tE 0
#define rad_tolerance 0.3
#define m_tolerance 0.1
enum states {
  LANDING_ZONE,
  NO_OBSTACLE,
  OBSTACLE,
  MISSION_SITE
};
states state = LANDING_ZONE;
boolean successful = false;

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
#define Offset -1.5            //deviation compensate
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

// Sponge Servo Set-Up
#define spongePin 11
Servo spongeServo;

// Timed Control Values Set-Up
int ms_m = 4255;  //milliseconds per meter
int ms_rad = 2045; //milliseconds per radian
int turn_offset = 150;  //how many ms faster a 90 degree turn is in a donut

void setup() {
  //Initialize Pins
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(echoPinR, INPUT);

  // Get Starting Coordinates
  updateCoordMS();
  while(!updateCoordOSV()){}
  
  // Drive to correct y-coord & turn East
  driveYCoordMS();
  enes.println("Made to y-coord first time");

  // Cross the rocky terrain
  driveForward(0.8*ms_m);
  while(!updateCoordOSV()){
      //loop until coordinates are updated
  }
  while(xPos < 1.2){
    driveForward(.1*ms_m);
    while(!updateCoordOSV()){
      //loop until coordinates are updated
    }
  }
  enes.println("Crossed the rocky terrain");

  // Reposition at the correct y-coord if off by more than 10cm
  if(max(yPos, yMS)-min(yPos, yMS) > 10){
    driveYCoordMS();
  }

  state = NO_OBSTACLE;
}

void loop() {
  // Wait until the OSV location has been updated
  while(!updateCoordOSV()){
    driveStop(0);
  }
  
  switch(state){
    case NO_OBSTACLE:
      enes.println("No Obstacle");
      if(!checkForObstacles()){
        if(xPos+.33+.15<xMS){
          driveForward(.15*ms_m);
        } else {
          driveForward((xMS-xPos-.33)*ms_m);
          state = MISSION_SITE;
        }
      } else {
        state = OBSTACLE;
      }
      break;
    case OBSTACLE:
      enes.println("Obstacle Detected");
      float tTurn;
      tTurn = turnToAvoidObstacle();
      driveAroundObstacle(tTurn, xPos, yPos);
      if(!checkForObstacles() && state != MISSION_SITE){
        state = NO_OBSTACLE;
      }
      break;
    case MISSION_SITE:
      enes.println("Mission Site");
      positionOSVatMS();
      enes.println("Positioned at the mission site!");
      //enes.navigated();
      missionSiteInit();
      readPH();
      while(!successful){
        retryMS();
      }
      break;
    default:
      enes.println("*** Error - Read Impossible State ***");
  }
}

/**
 * Adjusts the OSV's angle and coordinates to it is facing the
 * mission site and close enough to lower the pH sensor
 */
void positionOSVatMS(){
  enes.println("Positioning OSV at MS");
  double dx = xMS-xPos;
  double dy = max(yPos, yMS)-min(yPos, yMS);
  double tMS = atan(dy/dx);
  timedTurn(tMS);
  enes.println("facing MS");
  double dMS = sqrt(dx*dx+dy*dy);
  driveForward(dMS*ms_m);
  enes.println("drove to MS");
}

/**
 * Lowers the winch for 3 seconds and then stops
 */
void missionSiteInit(){
  winchServo.attach(winchPin);
  winchServo.write(130);
  delay(3000);
  //winchServo.write(90);
  winchServo.detach();
}

/**
 * Raises the winch, adjusts the OSV's positioning, and tries again
 */
void retryMS(){
  //raises winch
  winchServo.attach(winchPin);
  winchServo.write(50);
  delay(3000);
  winchServo.detach();
  positionOSVatMS();
  missionSiteInit();
  readPH();
}

/**
 * Reads and transmits the voltage & pH value
 * Code found online, slighly modified for our purpose
 * Updates successful based on pH reading
 */
void readPH(){
  boolean readPH = false;
  static float pHValue,voltage;
  while(!readPH){
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    if(millis()-samplingTime > samplingInterval)  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
    }
    //Every 800 milliseconds, print a numerical
    if(millis() - printTime > printInterval && readPH == false){   
      enes.baseObjective(pHValue);
      enes.print("Voltage:");
      enes.print(voltage);
      enes.print("    pH value: ");
      enes.println(pHValue);
      printTime=millis();
      readPH = true;
    }
  }
  if(pHValue < 7){ //TODO - is this a reasonable test of success?
    successful = true;
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
 * Drives 35cm North/South, 35cm East, and then back to original y-coord
 * Ends facing East
 */
void driveAroundObstacle(float tInit, float xInit, float yInit){
  enes.println("Driving around obstacle");
  // Drive N or S 50 cm
  driveForward(.35*ms_m);
  enes.println("Drove north / south 35 cm");
  // Drive E 50 cm
  enes.println("Turning East");
  while(max(tE, theta)-min(theta, tE)>rad_tolerance){
    timedTurn(tE);
    while(!updateCoordOSV()){
      //loop until coordinates are updated
    }
  }
  while(checkForObstacles()){
    enes.println("Still blocked by obstacle");
    timedTurn(tInit);
    driveForward(.35*ms_m);
    enes.println("Drove north / south 35 cm");
    enes.println("Turning East");
    while(max(tE, theta)-min(theta, tE)>rad_tolerance){
      timedTurn(tE);
      while(!updateCoordOSV()){
        //loop until coordinates are updated
      }
    }
  }
  driveForward(.5*ms_m);
  enes.println("Drove east 50 cm");
  // Return to MS y-coord and face E
  while(!updateCoordOSV()){
    //wait until location is updated
  }
  if(xPos>xMS || xMS-xPos<m_tolerance){
    state = MISSION_SITE;
  } else {
    driveYCoordMS();
    enes.println("Returned to ready position (yMS & E)");
  }
}

/**
 * Determins if on top or bottom half of arena & turns accordingly
 * towards the center
 * return - theta the OSV ends up facing
 */
float turnToAvoidObstacle(){
  driveBackward(.15*ms_m);
  if(yPos > 1){
    timedTurn(tS);
    return tS;
  } else {
    timedTurn(tN);
    return tN;
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
  //Determine if there is an obstacle within 20 cm
  if(dL <= 20 || dR <= 20){
    return true;
  }
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
 * Drives to the MS y-coord, then faces E
 * All driving is time-based, then the coordinates are updated 
 * & adjustments are made if necessary
 */
void driveYCoordMS(){
  float tCenter;
  if(yPos<yMS){ //south of MS
    tCenter = tN;
  } else { //north of MS
    tCenter = tS;
  }
  while(max(theta, tCenter)-min(theta, tCenter)>rad_tolerance){
    timedTurn(tCenter);
    while(!updateCoordOSV()){
      //loop until coordinates are updated
    }
  }
  updateCoordOSV();
  while(max(yPos, yMS)-min(yPos, yMS)>m_tolerance){
    long timeForward = (max(yPos, yMS)-min(yPos, yMS))*ms_m;
    driveForward(timeForward);
    while(!updateCoordOSV()){
      //loop until coordinates are updated
    }
  }
  while(max(tE, theta)-min(theta, tE)>rad_tolerance){
    timedTurn(tE);
    while(!updateCoordOSV()){
      //loop until coordinates are updated
    }
  }
}

/**
 * Turns to approximately theta based on a time constant
 */
void timedTurn(float angle){
  float delta = max(theta, angle)-min(theta, angle);
  long timeToTurn;
  if(delta<pi/2){
    timeToTurn = delta*ms_rad;
  } else {
    float over90 = (delta-pi/2)/(pi/2);
    timeToTurn = delta*ms_rad-over90*turn_offset;
  }
  if(angle>theta){
    driveLeft(timeToTurn);
  } else {
    driveRight(timeToTurn);
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
void driveForward(long time){
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, HIGH);
  digitalWrite(rm1, LOW);
  digitalWrite(rm2, HIGH);
  long start_time = millis();
  while(millis() < (start_time+time)){
    //delay for time miliseconds
  }
  driveStop(0);
}

/**
 * Drives forwards for a specified time
 * time = miliseconds, 0 for indefinitely
 */
void driveBackward(long time){
  digitalWrite(lm1, HIGH);
  digitalWrite(lm2, LOW);
  digitalWrite(rm1, HIGH);
  digitalWrite(rm2, LOW);
  long start_time = millis();
  while(millis() < (start_time+time)){
    //delay for time miliseconds
  }
  driveStop(0);
}

/**
 * Turns right for a specified time
 * time = miliseconds, 0 for indefinitely
 */
void driveRight(long time){
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, HIGH);
  digitalWrite(rm1, HIGH);
  digitalWrite(rm2, LOW);
  long start_time = millis();
  while(millis() < (start_time+time)){
    //delay for time miliseconds
  }
  driveStop(0);
}

/**
 * Turns left for a specified time
 * time = miliseconds, 0 for indefinitely
 */
void driveLeft(long time){
  digitalWrite(lm1, HIGH);
  digitalWrite(lm2, LOW);
  digitalWrite(rm1, LOW);
  digitalWrite(rm2, HIGH);
  long start_time = millis();
  while(millis() < (start_time+time)){
    //delay for time miliseconds
  }
  driveStop(0);
}

/**
 * Stops the wheel motors for a specified time
 * time = miliseconds, 0 for indefinitely
 */
void driveStop(long time){
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, LOW);
  digitalWrite(rm1, LOW);
  digitalWrite(rm2, LOW);
  long start_time = millis();
  while(millis() < (start_time+time)){
    //delay for time miliseconds
  }
}
