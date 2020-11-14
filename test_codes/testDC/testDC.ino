

void setup() {
  // put your setup code here, to run once:
  // Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

}

void loop() {
  // put your main code here, to run repeatedly:
  myMotor.setSpeed(100);
  myMotor.run(FORWARD);
  sleep(1000)
  myMotor.run(RELEASE);
  sleep(1000);
}
