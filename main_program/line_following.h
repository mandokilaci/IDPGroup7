int left_sensor_value = 0;
int right_sensor_value = 0;

//pins
int left_sensor = A1;
int right_sensor = A2;



void line_following(Adafruit_DCMotor *left_motor,Adafruit_DCMotor *right_motor, int yellow, int red){

left_motor -> setSpeed(100);
right_motor -> setSpeed(100);

left_sensor_value = analogRead(left_sensor);
right_sensor_value = analogRead(right_sensor);

Serial.print("left sensor value:");
Serial.print(left_sensor_value);

Serial.print("right sensor value:");
Serial.print(right_sensor_value);

/*if(left_sensor_value > 500 && right_sensor_value < 300){
    digitalWrite(yellow, HIGH);
    digitalWrite(red, HIGH);
    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);

    left_motor -> run(FORWARD);
    right_motor -> run(FORWARD);

    Serial.println("straight");
}

else*/ if(left_sensor_value < 300 && right_sensor_value > 500){
      digitalWrite(yellow, HIGH);
    digitalWrite(red, LOW);
    left_motor -> setSpeed(150);
    right_motor -> setSpeed(150);

    left_motor -> run(BACKWARD);
    right_motor -> run(FORWARD);

    Serial.println("left");
}

else if(left_sensor_value > 500 && right_sensor_value < 300){
      digitalWrite(yellow, LOW);
    digitalWrite(red, HIGH);
    left_motor -> setSpeed(150);
    right_motor -> setSpeed(150);

    left_motor -> run(FORWARD);
    right_motor -> run(BACKWARD);

    Serial.println("right");
}

else{
      digitalWrite(yellow, HIGH);
    digitalWrite(red, HIGH);
    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);

    left_motor -> run(FORWARD);
    right_motor-> run(FORWARD);

    Serial.println("continue straight");
}

return;

}
