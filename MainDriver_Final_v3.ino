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
#define tolerance 0.3
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

void setup() {
  //Initialize Pins
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);
  //attach and detach servo pins as needed to make sure the servo actually stops
  //winchServo.attach(winchPin);
  //spongeServo.attach(spongePin);
  pinMode(trigPinL, OUTPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(echoPinR, INPUT);


  // Get Coordinates
  updateCoordMS();
  updateCoordOSV();

  // Drive to correct y-coord & turn East
  driveYCoordMS();
  enes.println("Made to y-coord first time");

  // Drive across rocky terrain
  while(xPos < 1.2){
    driveForward(1);
    while(!updateCoordOSV()){
      driveStop(0);
    }
  }
  enes.println("Crossed the rocky terrain");

  // Readjust to correct y-coord & facing East
//  driveYCoordMS();
//  enes.println("Made to y-coord second time");  

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
      enes.println("No Obstacle");
      if(xPos < xMS-.2){
        driveForward(1);
        checkForObstacles();
      } else {
        state = MISSION_SITE;
      }
      break;
    case OBSTACLE:
      enes.println("Obstacle");
      //TODO - account for if only one sensor found an obstacle
      float tTurn;
      tTurn = turnToAvoidObstacle();
      driveAroundObstacle(tTurn, xPos, yPos);
      checkForObstacles();
      //TODO - what if it made it to the MS in the middle of this?
      break;
    case MISSION_SITE:
      enes.println("Mission Site");
      positionOSVatMS();
      enes.println("Made it to the mission site!");
      //enes.navigated();
      missionSiteInit();
      collectWater();
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
 * Determines if there are any obstacles within 15 cm and
 * adjusts the state accordingly
 * return true if obstacle detected, otherwise false
 */
boolean checkForObstacles(){
  //Get the distance each sensor is detecting
  int dL = getDistance(trigPinL, echoPinL);
  int dR = getDistance(trigPinR, echoPinR);
  //Determine if there is an obstacle within 15 cm
  if(dL <= 20 || dR <= 20){
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
 * Drives 35cm North/South, 35cm East, and then back to original y-coord
 * Ends facing East
 */
void driveAroundObstacle(float tInit, float xInit, float yInit){
  enes.println("Driving around obstacle");
  if(tInit == tN){ //facing North
    enes.println("Facing North");
    // Drive N 50 cm
    enes.print("yPos: ");
    enes.print(yPos);
    enes.print("  yInit: ");
    enes.println(yInit);
    while(yPos < yInit+.5){
      driveForward(1);
      while(!updateCoordOSV()){
        driveStop(0);    
      }
    }
    enes.println("Drove north 50 cm");
  } else { //facing South
    enes.println("Facing South");
    // Drive S 50 cm
    enes.print("yPos: ");
    enes.print(yPos);
    enes.print("  yInit: ");
    enes.println(yInit);
    while(yPos > yInit-.5){
      driveForward(1);
      while(!updateCoordOSV()){
        driveStop(0);   
      } 
    }
    enes.println("Drove south 50 cm");
  }
  // Drive E 50 cm
  enes.println("Turning East");
  turnToTheta(tE);
  while(xPos < xInit+.5){
    driveForward(1);
    while(!updateCoordOSV()){
      driveStop(0);    
    }
  }
  enes.println("Drove east 50 cm");
  // Return to MS y-coord and face E
  driveYCoordMS();
  enes.println("Returned to ready position (yMS & E)");
}

/**
 * Adjusts the OSV's angle and coordinates to it is facing the
 * mission site and close enough to lower the pH sensor
 */
void positionOSVatMS(){
  //TODO
  //ideas: calculate theta to face MS and turn to that angle
    //drive until sensors detect onstacle within 2 cm
  // Calculate theta to face MS
  double dx = xMS-xPos;
  double dy = max(yPos, yMS)-min(yPos, yMS);
  double tMS = atan(dy/dx);
  turnToTheta(tMS);
  //Get the distance each sensor is detecting
  int dL = getDistance(trigPinL, echoPinL);
  int dR = getDistance(trigPinR, echoPinR);
  //Determine if the mission site is within 2 cm
  while(dL > 4 && dR > 4){
    driveForward(1);
    dL = getDistance(trigPinL, echoPinL);
    dR = getDistance(trigPinR, echoPinR);
    driveStop(0);
  }
}

/**
 * Lowers the winch for 1.5 seconds and then stops
 */
void missionSiteInit(){
  winchServo.attach(winchPin);
  winchServo.write(130);
  delay(1500);
  //winchServo.write(90);
  winchServo.detach();
}

void collectWater(){
  spongeServo.attach(spongePin);
  spongeServo.write(120);
  delay(1000);
  spongeServo.detach();
  delay(5000);
  spongeServo.attach(spongePin);
  spongeServo.write(65);
  delay(1000);
  spongeServo.detach();
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
    if(millis() - printTime > printInterval){   
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
 * Raises the winch, adjusts the OSV's positioning, and tries again
 */
void retryMS(){
  //raises winch
  winchServo.attach(winchPin);
  winchServo.write(50);
  delay(1500);
  winchServo.write(90);
  winchServo.detach();
  //TODO - adjusts position
  //lowers winch and tries to read the pH again
  missionSiteInit();
  readPH();
}

/**
 * Drives the OSV straight North ot South to the MS
 * y-coordinate, then turns East
 */
void driveYCoordMS(){
  //if South of MS y-coord
  if(yPos<yMS){
    enes.println("Am south of mission site");
    if(max(theta, tN)-min(theta,tN) > tolerance){
      turnToTheta(tN); //face North
      enes.print("Facing north w/ theta ");
      enes.println(theta);
    }
    //rough positioning
    while(yPos<yMS-.3){
      driveForward(1);
      while(!updateCoordOSV()){
          driveStop(0);
      }
    }
    //fine tuning
    while(yPos<yMS){
      driveForward(1);
      driveStop(50);
      while(!updateCoordOSV()){
          driveStop(0);
      }
    }
  }
  driveStop(0);
  //if North of MS y-coord
  if(yPos>yMS){
    enes.println("Am north of mission site");
    if(max(theta, tS)-min(theta,tS) > tolerance){
      turnToTheta(tS); //face South
      enes.print("Facing south w/ theta ");
      enes.println(theta);
    }
    //rough positioning
    while(yPos>yMS+.3){
      driveForward(1);
      while(!updateCoordOSV()){
          driveStop(0);
      }
    }
    //fine tuning
    while(yPos>yMS){
      driveForward(1);
      driveStop(50);
      while(!updateCoordOSV()){
          driveStop(0);
      }
    }
  }
  driveStop(0);
  //at correct y-coord - now turn East
  enes.println("At the MS y-coord!");
  turnToTheta(tE);
  enes.println("Facing East");
}

/**
 * Turns the OSV until it is facing the inputed direction
 * angle = desired direction in radians
 */
void turnToTheta(float angle){
  /*float delta = max(theta, angle) - min(theta, angle);
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
  } */
  // Simplified version
  float delta = max(theta, angle) - min(theta, angle);
  if((angle>theta && delta<pi) || (theta>angle && delta>pi)){
    // Turn to roughly the right angle
    while((max(theta, angle)-min(theta, angle)) > 0.3){
      driveLeft(1);
      while(!updateCoordOSV()){
            driveStop(0);
      }
     // enes.print("Theta: ");
      //enes.println(theta);
     // enes.print("Delta: ");
     // enes.println(max(theta, angle)-min(theta, angle));
      driveStop(0); //try to make it turn slower
    }
    // Slowly fine tune the turn angle
    while((max(theta, angle)-min(theta, angle)) > 0.1){
      driveLeft(1);
      driveStop(50);
      while(!updateCoordOSV()){
            driveStop(0);
      }
    }
    enes.print("Turned to theta!   ");
    enes.println(theta);
    driveStop(0);
    //WORKED :D
  }
  if((theta>angle && delta<pi) || (angle>theta && delta>pi)){
    // Turn to roughly the right angle
    while((max(theta, angle)-min(theta, angle)) > 0.3){
      driveRight(1);
      while(!updateCoordOSV()){
            driveStop(0);
      }
      //enes.print("Theta: ");
     // enes.println(theta);
      //enes.print("Delta: ");
      //enes.println(max(theta, angle)-min(theta, angle));
      driveStop(0); //try to make it turn slower
    }
    // Slowly fine tune the turn angle
    while((max(theta, angle)-min(theta, angle)) > 0.1){
      driveRight(1);
      driveStop(50);
      while(!updateCoordOSV()){
            driveStop(0);
      }
    }
    enes.print("Turned to theta!    ");
    enes.println(theta);
    driveStop(0);
    //WORKED :D
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
