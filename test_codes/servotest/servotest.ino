/*
  Arduino Starter Kit example
  Project 5 - Servo Mood Indicator

  This sketch is written to accompany Project 5 in the Arduino Starter Kit

  Parts required:
  - servo motor
  - 10 kilohm potentiometer
  - two 100 uF electrolytic capacitors

  created 13 Sep 2012
  by Scott Fitzgerald

  http://www.arduino.cc/starterKit

  This example code is part of the public domain.
*/

// include the Servo library
#include <Servo.h>

Servo myServo;  // create a servo object

int angle;   // variable to hold the angle for the servo motor
int pos;
void setup() {
  myServo.attach(9); // attaches the servo on pin 9 to the servo object
  Serial.begin(9600); // open a serial connection to your computer
}

void loop() {
  //angle=Serial.read()+30;

  // print out the angle for the servo motor
  Serial.print(", angle: ");
  Serial.println(angle);

  // set the se1rvo position
  myServo.write(40);
    delay(1500);
  for (pos = 40; pos <= 120; pos += 5) { // goes from 0 degrees to 180 degrees
        // in steps of 5 degree
        myServo.write(pos);              // tell servo to go to position in variable 'pos'
        Serial.print(pos);
        delay(150);                       // waits 15ms for the servo to reach the position
  }
  // wait for the servo to get there
  delay(1500);
}
