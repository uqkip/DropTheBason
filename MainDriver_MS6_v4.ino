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
boolean passedRockyTerrain = false;

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


void setup() {
  //Initialize Pins
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);
  winchServo.attach(winchPin);
  winchServo.write(90);

  // Temporary Init Code
  missionSiteInit();

  // Final Init Code
  
}

void loop() {
  // Temporary Loop Code
  missionSiteLoop();

  // Final Loop Code
}

/**
 * Retrieves the OSV and mission site coordinates and
 * drives to the mission site y-coord, then turn East
 */
void milestone6init(){
  updateCoordMS();
  updateCoordOSV();
  driveYCoordMS();
}

/**
 * Drives East until arriving at the mission site
 * Adjusts positioning immediately after the rocky terrain
 */
void milestone6loop(){
  // Recalibrate y-coord after passing the rocky terrain
  if(passedRockyTerrain == false && xPos > 1.2){
    if(max(yMS, yPos)-min(yMS, yPos) > .1){
      driveYCoordMS();
    }
    else if(max(theta, tE)-min(theta, tE) > tolerance){
      turnToTheta(tE);
    }
    passedRockyTerrain = true;
  }
  // Continue East until OSV is ~20 cm from mission site
  if(xPos < xMS-.4){
    driveForward(1);
    while(!updateCoordOSV()){
      driveStop(0);
    }
  } else {
    driveStop(0);
    enes.println("Made it to Mission Site!");
  }
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
