#include<Adafruit_MotorShield.h>

#include "line_following3.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *left_motor = AFMS.getMotor(1);
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);

//test LEDs
int yellow=4;
int red=7;

//pins
const int btn=2; //start/stop button
const int lls = 3; //line sensors
const int mls = 4;
const int rls = 5;

bool running=false;

void setup() {
  // put your setup code here, to run once:

  AFMS.begin();

  Serial.begin(9600);
pinMode(lls, INPUT);
pinMode(mls, INPUT);
pinMode(rls, INPUT);
pinMode(btn, INPUT);
attachInterrupt(digitalPinToInterrupt(btn), blink, RISING);
}

void blink() {
  running = !running;
}

void loop() {
  // put your main code here, to run repeatedly:
  while(running){
    long start = millis();
    long end = start;

    while((end - start) < 1000){
      line_following(left_motor,right_motor,lls,mls,rls);
      end = millis();
    }
  }

}
