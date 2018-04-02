// The motor turns one direction for 1 sec, the opposite direction for 1 sec,
// and then stops for 1 sec, indefinitely

/**
 * Hook up the arduino to the motor controller with the following connections:
 * Digital7 --> in1
 * Digital6 --> in2
 * Motor A connected to motor controller
 */

#define lm1 7
#define lm2 6
#define rm1 5
#define rm2 4

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);
}

void loop() {
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, HIGH);
  delay(1000);
  digitalWrite(lm1, LOW);
  digitalWrite(lm2, LOW);
  delay(1000);
  digitalWrite(lm1, HIGH);
  digitalWrite(lm2, LOW);
  delay(1000);
}
