int left_sensor_value = 0;
int right_sensor_value = 0;
int middle_sensor_value = 0;

void line_following(Adafruit_DCMotor *left_motor, Adafruit_DCMotor *right_motor,int left_sensor, int middle_sensor, int right_sensor){

left_motor -> setSpeed(100);
right_motor -> setSpeed(100);

left_sensor_value = digitalRead(left_sensor);
right_sensor_value = digitalRead(right_sensor);
middle_sensor_value = digitalRead(middle_sensor);

Serial.print("left sensor value:");
Serial.print(left_sensor_value);

Serial.print("middle sensor value:");
Serial.print(middle_sensor_value);

Serial.print("right sensor value:");

if (left_sensor_value==HIGH && right_sensor_value==HIGH && middle_sensor_value==LOW){

    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);

    left_motor -> run(BACKWARD);
    right_motor -> run(BACKWARD);

    Serial.println("straight");
}
//T junction
else if (left_sensor_value==LOW && right_sensor_value==LOW && middle_sensor_value==LOW){

    left_motor -> setSpeed(150);
    right_motor -> setSpeed(150);

    left_motor -> run(FORWARD);
    right_motor -> run(FORWARD);

    Serial.println("T junction");
}

else if (left_sensor_value==HIGH && right_sensor_value==LOW && middle_sensor_value==HIGH){

    left_motor -> setSpeed(150);
    right_motor -> setSpeed(150);

    left_motor -> run(BACKWARD);
    right_motor -> run(FORWARD);

    Serial.println("left");
}
else if (left_sensor_value==LOW && right_sensor_value==HIGH && middle_sensor_value==HIGH){

    left_motor -> setSpeed(150);
    right_motor -> setSpeed(150);

    left_motor -> run(FORWARD);
    right_motor -> run(BACKWARD);

    Serial.println("right");
}
else{

    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);

    left_motor -> run(FORWARD);
    right_motor-> run(FORWARD);

    Serial.println("error");
}

return;

}
