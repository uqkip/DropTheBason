#include <Servo.h>

Servo myservo;  

void setup() 
{
  myservo.attach(7);  // attaches the servo on pin 7 to the servo object
  myservo.write(90);   // Servo should be stopped initially
}

void loop() 
{
//Dropping the meter in
  myservo.write(100); // dropping pH meter
  delay(4500);         // 4.5 second spin 
  myservo.write(90);   //stop
  delay(5000);         // PLACEHOLDER, without this, servo will keep spinning
                       // Need to exit loop
}



