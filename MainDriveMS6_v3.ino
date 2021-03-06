/**
 * ENES 100 0501 Chemical Team
 * Drop the Bason
 * 
 * Created 3/26/18
 * 
 * Creator: Kendall Price
 * Contributors: 
 */

/**
 * USEFUL NOTES for REFERENCE
 * - sensors have same pulse different echos
 * - 
 */

#include "Enes100.h"
#include <Servo.h>

#define lm1 7
#define lm2 6
#define rm1 5
#define rm2 4

#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset -0.3            //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
int winchPin = 10;

Enes100 enes("Drop the Bason", CHEMICAL, 19, 8, 9); //teamName, teamType, markerID, rxPin, txPin
/*enum state{
  LANDING,
  ROCKY,
  CORRECT_Y,
  OBSTACLE,
  WRONG_Y,
  MISSION_SITE
};*/
// x from 0 to 4m, y from 0 to 2m, theta from -pi to pi rad
float xMS, yMS, xPos, yPos, theta;

const float pi = 3.14;
float tN = 1.57;
float tS = -1.57;
float tE = 0;
float tW1 = 3.14, tW2 = -3.14;

float tolerance = 0.01;
int loops;

float dD = 0; //dD is the diagonal displacement of the RF marker from center of OSV
float phi = 0; //phi is the angle that dD makes with the outer edge of OSV in rad

Servo winchServo;

void setup() {
  /* put your setup code here, to run once: */
  
  // Initializa values
  //state = LANDING; //TODO WHY IS THIS AN ERROR?
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);

  winchServo.attach(winchPin);
  winchServo.write(90);

  //milestone6init();
  
  // Retreive mission coordinates
  /*while (!enes.retrieveDestination()) {
    enes.println(" *** Unable to retrieve mission site coordinates *** ");
  }
  xMS = enes.destination.x;
  yMS = enes.destination.y;
  enes.print("Mission site at (");
  enes.print(xMS);
  enes.print(", ");
  enes.print(yMS);
  enes.println(+ ")");*/

/*
  // Get starting position
 while(!enes.updateLocation()){
    enes.println(" *** Unable to retrieve starting location. ***");
  }
  xPos = enes.location.x;
  yPos = enes.location.y;
  theta = enes.location.theta;
  */
  
  //milestone5movement();
  missionSite();
}

void loop() {
  /* put your main code here, to run repeatedly: */
  //milestone6loop();
  
  /*updateCoordOSV();

  enes.print("x-coord: ");
  enes.println(xPos);
  enes.print("y-coord: ");
  enes.println(yPos);
  enes.print("theta: ");
  enes.println(theta);*/

  // Don't move unless you have updated the location
  /*if (!enes.updateLocation()){
    enes.println(" *** Unable to update location. ***");
  } else {
    // Update current position
    xPos = enes.location.x;
    yPos = enes.location.y;
    theta = enes.location.theta;
    
    //movementCases();    
  }*/

  //milestone5communication();
  
  // Switch w/ cases:
    // 1 - OSV is in landing zone
      // rotate to face correct NS direction
      // drive towards correct y-coord
      // if at correct y-coord, switch to state 2
    // 2 - OSV is crossing the rocky terrain
      // rotate to face E
      // if not all the way over the rocky terrain, continue driving
      // if completely over the rocky terrain, switch to state 3
    // 3 - OSV is in "easy" transit to mission site
      // if you see an obstacle: switch to state 4
      // if at the wrong y-coord, switch to state 5
      // if at cmission site x-coord, state 6
      // otherwise, drive forwards
    // 4 - OSV encountered an obstable
      // turn to face NS based on half of arena
      // check for new obstacle
        // if yes, turn 180 and check again
          // if yes AGAIN. turn E and drive 35 cm
          // then face NS (decision) and drive 70 cm
        // if no
          // drive 35 cm N/S and then turn W
      // turn to face W
      // switch to state 3
    // 5 - OSV at wrong y-coord
      // check for obstacle
        // if no: drive 35 cm forward
        // if yes: switch to state 4 and exit
      // turn NS towards correct y-coord
      // if obstacle not detected
        // drive to correct y-coord and  and turn W
        // switch to state 3
      // else face W
    // 6 - OSV is at mission site
  
  /*if(state = MISSION_SITE){
    missionSite()
  }*/

  
}

void milestone6init(){
  updateCoordMS();
  updateCoordOSV();
  driveYCoordMS();
  loops = 0;
  /*turnToTheta(tN);
  enes.print("Facing North! Theta is ");
  enes.println(theta);
  delay(2000);
  turnToTheta(tE);
  enes.print("Facing East! Theta is ");
  enes.println(theta);
  delay(2000);
  turnToTheta(tS);
  enes.print("Facing South! Theta is ");
  enes.println(theta);
  delay(1000);
  turnToTheta(tW1);
  enes.print("Facing West! Theta is ");
  enes.println(theta);
  delay(1000);
  turnToTheta(tW2);
  enes.print("Facing West! Theta is ");
  enes.println(theta);
  delay(1000);
  turnToTheta(tN);
  enes.print("Facing North! Theta is ");
  enes.println(theta); */
}

void milestone6loop(){
  loops = loops+1;
  //if(max(theta, tE)-min(theta, tE) > .1){ //angle off by more than a degree
  if(loops%500 == 0){
    if(max(yMS, yPos)-min(yMS, yPos) > .1){
      driveYCoordMS();
    }
    else if(max(theta, tE)-min(theta, tE) > 3*tolerance){
      turnToTheta(tE);
    }
  }
  if(xPos < xMS-.4){ //stop 20cm from mission site
    driveForward(1);
    while(!updateCoordOSV()){
      driveStop(0);
    }
  } else {
    driveStop(0);
    enes.print("Made it to Mission Site!");
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
    
    /*if(theta >= tE && theta <= tN){ //first quadrant
      alpha = phi-theta;
      xPos = xPosRF+dD*sin(alpha);
      yPos = yPosRF-dD*cos(alpha);
    } else if(theta >= tN){ //second quadrant
      alpha = theta-phi-tN;
      xPos = xPosRF+dD*sin(alpha);
      yPos = yPosRF+dD*cos(alpha);
    } else if (theta <= tE && theta >= tS) { //third quadrant
      alpha = 2*tN-phi+theta;
      xPos = xPosRF-dD*cos(alpha);
      yPos = yPosRF-dD*sin(alpha);
    } else {
      alpha = 2*tN-phi+theta;
      xPos = xPosRF-dD*cos(alpha);
      yPos = yPosRF-dD*sin(alpha);
    }*/
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


void missionSite(){
  winchServo.write(100);
  delay(4500);
  winchServo.write(90);
  winchServo.detach();
  //read & transmit pH value
  readAndTransmitPH();
}

void readAndTransmitPH(){
  while(true){
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    static float pHValue,voltage;
    if(millis()-samplingTime > samplingInterval)
    {
        pHArray[pHArrayIndex++]=analogRead(SensorPin);
        if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
        voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
        pHValue = 3.5*voltage+Offset;
        samplingTime=millis();
    }
    if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
    {
    enes.print("Voltage:");
          enes.print(voltage);
          enes.print("    pH value: ");
    enes.println(pHValue);
          printTime=millis();
    }
  }
}

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
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

/*
void milestone5movement(){
  driveForward(15500);
  driveStop(2000);
  driveBackward(15500);
  driveStop(2000);

  int timeToTurn = 3212;
  driveRight(timeToTurn);
  driveStop(2000);
  driveRight(timeToTurn-150);
  driveStop(2000);
  driveRight(timeToTurn-150);
  driveStop(2000);
  driveLeft(3*timeToTurn-300);
  driveStop(0);
}

void milestone5communication(){
  enes.print("The mission site is ");
  if(yPos < yMS){
    enes.print("North ");
  } else if (yPos > yMS){
    enes.print("South ");
  }
  if(xPos < xMS){
    enes.print("East ");
  } else if (xPos > xMS){
    enes.print("West ");
  }
  enes.println("of the OSV");
}
*/
