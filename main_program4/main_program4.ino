#include<Adafruit_MotorShield.h>

#include "line_following4.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *left_motor = AFMS.getMotor(1);
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);

//pins
const int going = 13;//flashing
const int lr = A2;
const int lb = A3; 


const int btn=2; //start/stop button
const int lls = 3; //line sensors
const int mlls = 4;
const int mrls = 5;
const int rls = 6;

bool running=false;

void setup() {
  // put your setup code here, to run once:

  AFMS.begin();

  Serial.begin(9600);
  pinMode(lls, INPUT);
  pinMode(mlls, INPUT);
  pinMode(mrls, INPUT);
  pinMode(rls, INPUT);
  pinMode(btn, INPUT);
  attachInterrupt(digitalPinToInterrupt(btn), blink, RISING);
}

void blink() {
  running = !running;
}

void loop() {
   if(!running){
      left_motor -> run(RELEASE);
      right_motor -> run(RELEASE);
      digitalWrite(going, LOW);
      digitalWrite(lr, LOW);
      digitalWrite(lb, LOW);
      Serial.println("not running");
      delay(100);//TODO not quite good
  }
  // put your main code here, to run repeatedly:
  while(running){
    digitalWrite(going, HIGH);
    long start = millis();
    long end = start;

    while((end - start) < 1000){
      line_following(left_motor,right_motor,lls,mlls,mrls,rls);
      end = millis();
    }
  }

}
