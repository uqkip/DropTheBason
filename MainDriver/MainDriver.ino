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

#define lm1 7
#define lm2 6
#define rm1 5
#define rm2 4

Enes100 enes("Drop the Bason", CHEMICAL, 19, 8, 9); //teamName, teamType, markerID, rxPin, txPin
enum state{
  LANDING,
  ROCKY,
  CORRECT_Y,
  OBSTACLE,
  WRONG_Y,
  MISSION_SITE
};
// x from 0 to 4m, y from 0 to 2m, theta from -pi to pi rad
float xMS, yMS, xPos, yPos, theta;

void setup() {
  /* put your setup code here, to run once: */
  
  // Initializa values
  state = LANDING; //TODO WHY IS THIS AN ERROR?
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);
  
  // Retreive mission coordinates
  while (!enes.retrieveDestination()) {
    enes.println(" *** Unable to retrieve mission site coordinates *** ");
  }
  xMS = enes.destination.x;
  yMS = enes.destination.y;
  enes.println("Mission site at ("+xMS+", "+yMS+")");

  // Get starting position
  while(!enes.updateLocation()){
    enes.println(" *** Unable to retrieve starting location. ***");
  }
  xPos = enes.location.x;
  yPos = enes.location.y;
  theta = enes.location.theta;

  milestone5movement();
}

void loop() {
  /* put your main code here, to run repeatedly: */
  
  // Don't move unless you have updated the location
  if (!enes.updateLocation()){
    enes.println(" *** Unable to update location. ***");
  } else {
    // Update current position
    xPos = enes.location.x;
    yPos = enes.location.y;
    theta = enes.location.theta;

    movementCases();    
  }
  
  if(state = MISSION_SITE){
    missionSite()
  }

  milestone5communication();
}

void movementCases(){
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
}

void missionSite(){
  
}

void milestone5movement(){
  driveForward(1000);
  driveStop(1000);
  driveBackward(1000);
  driveStop(1000);
  
  driveRight(250);
  driveStop(1000);
  driveRight(250);
  driveStop(1000);
  driveRight(250);
  driveStop(1000);
  driveLeft(750);
}

void milestone5communication(){
  String direction = "";
  if(yPos < yMS){
    direction = direction+"North ";
  } else if (yPos > yMS){
    direction = direction+"South ";
  }
  if(xPos < xMS){
    direction = direction+"East ";
  } else if (xPos > xMS){
    direction = direction+"West ";
  }
  enes.println("The mission site is "+direction+"of the OSV");
}

/**
 * Drives forwards
 * time = miliseconds
 */
void driveForward(int time){
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, HIGH);
  digitalWrite(2m1, LOW);
  digitalWrite(2m2, HIGH);
  delay(time);
}

/**
 * Drives forwards
 * time = miliseconds
 */
void driveBackward(int time){
  digitalWrite(lm1, HIGH);
  digitalWrite(lm2, LOW);
  digitalWrite(2m1, HIGH);
  digitalWrite(2m2, LOW);
  delay(time);
}

/**
 * Turns right
 * time = miliseconds
 */
void driveRight(int time){
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, HIGH);
  digitalWrite(2m1, HIGH);
  digitalWrite(2m2, LOW);
  delay(time);
}

/**
 * Turns left
 * time = miliseconds
 */
void driveLeft(int time){
  digitalWrite(lm1, HIGH);
  digitalWrite(lm2, LOW);
  digitalWrite(2m1, LOW);
  digitalWrite(2m2, HIGH);
  delay(time);
}

/**
 * Stops the wheel motors for a specified time
 * time = miliseconds
 */
void driveStop(int time){
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, LOW);
  digitalWrite(2m1, LOW);
  digitalWrite(2m2, LOW);
  delay(time);
}

void turnToTheta(float angle){
  while(theta != angle){ //make this have a little more leeaway (maybe convert to deg. and do whole deg?)
    //see which way too turn
    // turn a little
  }
}

