#include<Adafruit_MotorShield.h>

#include "line_following.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *left_motor = AFMS.getMotor(1);
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);

//test LEDs
int yellow=4;
int red=7;

void setup() {
  // put your setup code here, to run once:

  AFMS.begin();

  Serial.begin(9600);
pinMode(yellow, OUTPUT);
pinMode(red, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  long start = millis();
  long end = start;

  while((end - start) < 1000){
    line_following(left_motor,right_motor,yellow,red);
    end = millis();
  }

}
