int left_sensor_value = 0;
int right_sensor_value = 0;

//pins
int left_sensor = A1;
int right_sensor = A2;

<<<<<<< HEAD


void line_following(Adafruit_DCMotor *left_motor,Adafruit_DCMotor *right_motor, int yellow, int red){
=======
void line_following(Adafruit_DCMotor *left_motor,Adafruit_DCMotor *right_motor){
>>>>>>> 8c726ab065a4cf77aa65b6165eca5f82b8b16cd1

left_motor -> setSpeed(100);
right_motor -> setSpeed(100);

left_sensor_value = analogRead(left_sensor);
right_sensor_value = analogRead(right_sensor);

Serial.print("left sensor value:");
Serial.print(left_sensor_value);

Serial.print("right sensor value:");
Serial.print(right_sensor_value);

<<<<<<< HEAD
/*if(left_sensor_value > 500 && right_sensor_value < 300){
    digitalWrite(yellow, HIGH);
    digitalWrite(red, HIGH);
=======
if(left_sensor_value > 500 && right_sensor_value < 300){
>>>>>>> 8c726ab065a4cf77aa65b6165eca5f82b8b16cd1
    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);

    left_motor -> run(FORWARD);
    right_motor -> run(FORWARD);

    Serial.println("straight");
}

<<<<<<< HEAD
else*/ if(left_sensor_value < 300 && right_sensor_value > 500){
      digitalWrite(yellow, HIGH);
    digitalWrite(red, LOW);
=======
else if(left_sensor_value < 300 && right_sensor_value > 500){
>>>>>>> 8c726ab065a4cf77aa65b6165eca5f82b8b16cd1
    left_motor -> setSpeed(150);
    right_motor -> setSpeed(150);

    left_motor -> run(BACKWARD);
    right_motor -> run(FORWARD);

    Serial.println("left");
}

else if(left_sensor_value > 500 && right_sensor_value < 300){
<<<<<<< HEAD
      digitalWrite(yellow, LOW);
    digitalWrite(red, HIGH);
=======
>>>>>>> 8c726ab065a4cf77aa65b6165eca5f82b8b16cd1
    left_motor -> setSpeed(150);
    right_motor -> setSpeed(150);

    left_motor -> run(FORWARD);
    right_motor -> run(BACKWARD);

    Serial.println("right");
}

else{
<<<<<<< HEAD
      digitalWrite(yellow, HIGH);
    digitalWrite(red, HIGH);
=======
>>>>>>> 8c726ab065a4cf77aa65b6165eca5f82b8b16cd1
    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);

    left_motor -> run(FORWARD);
    right_motor-> run(FORWARD);

    Serial.println("continue straight");
}

return;

}
