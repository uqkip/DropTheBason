# DropTheBason
ENES100 0501 Chemical Team

Upload the MainDriver to the arduino uno and run to complete the chemical mission.

## Work in Progress

Current Tasks
- Understand how methods and functions work in arduino in order to develop structure
- Work on mini projects to develop background knowledge for coding the OSV (priority #1 is motors)

Mini Projects to Help Understand Using the Arduino:

  ~ Activate a (wheel) motor through the arduino & motor controller
  ~ Activate and then reverse the direction of a motor
  ~ Activate a motor for a specified number of rotations (or time?)
  ~ Activate multiple motors

  ~ Recieve input from the ultrasonic sensor
  ~ Have the ultrasonic sensor detact an obstacle
  ~ Have the ultrasonic sensor detect an obstacle only within 10cm
  ~ Receive input from multiple ultrasonic sensors

  ~ Activate a servo motor through the arduino
  ~ Activate the servo for a specified (fractional) number of turns
  
  ~ Recieve input from the pH sensor through the arduino

Resources:
https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
https://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/


Ideas for Solving RF Issues
- Make the RF marker a little larger (keep the white border) and make sure all the black is completely dark
- Set the course brightness on the mission control monitor lower
- Have the OSV move slower

Ideas for Solving turToTheta Issues
- Make the OSV turn slower so it's less likely to overshoot
- Check if it was successful at the end of the method and do recursion until correct
