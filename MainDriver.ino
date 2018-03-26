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

int state = 0;

void setup() {
  /* put your setup code here, to run once: */
  
  // Initializa values
  state = 1;
  // Retreive mission coordinates
  // Find starting location
  // Find starting direction

  // SAMPLE: Hello World
  /*
  Serial.begin(9600);
  Serial.print("Hello world");
  */
}

void loop() {
  /* put your main code here, to run repeatedly: */
  
  // Update current position
  
  // Switch w/ cases:
    // 1 - OSV is in landing zone
      // rotate to face correct NS direction
      // drive towards correct y-coord
      // if at correct y-coord, switch to state 2
    // 2 - OSV is crossing the rocky terrain
      // rotate to face E
      // if not over the rocky terrain, continue driving
      // if over the rocky terrain, switch to state 3
    // 3 - OSV is in "easy" transit to mission site
      // if you see an obstacle: switch to state 4
      // otherwise, drive forwards
      // if at mission site x-coord, state 5
    // 4 - OSV encountered an obstable
    // 5 - OSV is at mission site
}
