#include<Adafruit_MotorShield.h>

#include "line_following.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *left_motor = AFMS.getMotor(1);
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);

<<<<<<< HEAD
//test LEDs
int yellow=4;
int red=7;
=======
>>>>>>> 8c726ab065a4cf77aa65b6165eca5f82b8b16cd1

void setup() {
  // put your setup code here, to run once:

  AFMS.begin();

  Serial.begin(9600);
<<<<<<< HEAD
pinMode(yellow, OUTPUT);
pinMode(red, OUTPUT);
=======

>>>>>>> 8c726ab065a4cf77aa65b6165eca5f82b8b16cd1
}

void loop() {
  // put your main code here, to run repeatedly:

  long start = millis();
  long end = start;

  while((end - start) < 1000){
<<<<<<< HEAD
    line_following(left_motor,right_motor,yellow,red);
=======
    line_following(left_motor,right_motor);
>>>>>>> 8c726ab065a4cf77aa65b6165eca5f82b8b16cd1
    end = millis();
  }

}
