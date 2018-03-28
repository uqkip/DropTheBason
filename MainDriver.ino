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

Enes100 enes("Drop the Bason", CHEMICAL, 19, 8, 9); //teamName, teamType, markerID, rxPin, txPin
enum state{
  LANDING,
  ROCKY,
  CORRECT_Y,
  OBSTACLE,
  WRONG_Y,
  MISSION_SITE
};
float xMS, yMS, xPos, yPos, theta;

void setup() {
  /* put your setup code here, to run once: */
  
  // Initializa values
  state = LANDING;
  
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
  
  // SAMPLE: Hello World
  /*
  Serial.begin(9600);
  Serial.print("Hello world");
  */
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
  
  // mission site subroutine is not depended on location updating
}
