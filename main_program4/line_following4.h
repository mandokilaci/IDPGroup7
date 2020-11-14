int vlls, vmlls, vmrls, vrls;

void line_following(Adafruit_DCMotor *left_motor, Adafruit_DCMotor *right_motor,int lls, int mlls, int mrls, int rls){

left_motor -> setSpeed(100);
right_motor -> setSpeed(100);

vlls=digitalRead(lls);
vmlls=digitalRead(mlls);
vmrls=digitalRead(mrls);
vrls=digitalRead(rls);


Serial.print("left sensor value:");
Serial.print(vlls);

Serial.print("middle sensor value:");
Serial.print(vmlls);
Serial.print(vmrls);
Serial.print("right sensor value:");
Serial.print(vrls);

if (!vlls&&!vmlls&&!vmrls&&!vrls){

    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);

    left_motor -> run(BACKWARD);
    right_motor -> run(BACKWARD);

    Serial.println("straight");
}
//T junction
else if (vlls&&vmlls&&vmrls&&vrls){

    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);

    left_motor -> run(BACKWARD);
    right_motor -> run(BACKWARD);

    Serial.println("T junction");
}

else if (!vlls&&!vmlls&&vmrls&&!vrls){

    left_motor -> setSpeed(50);
    right_motor -> setSpeed(100);

    left_motor -> run(BACKWARD);
    right_motor -> run(BACKWARD);

    Serial.println("slightly left");
}
else if (!vlls&&vmlls&&!vmrls&&!vrls){

    left_motor -> setSpeed(100);
    right_motor -> setSpeed(50);

    left_motor -> run(BACKWARD);
    right_motor -> run(BACKWARD);

    Serial.println("slightly right");
    
} else if (!vlls&&!vmlls&&vmrls&&vrls){

    left_motor -> setSpeed(50);
    right_motor -> setSpeed(50);

    left_motor -> run(FORWARD);
    right_motor -> run(BACKWARD);

    Serial.println("left");

} else if (vlls&&vmlls&&!vmrls&&!vrls){

    left_motor -> setSpeed(50);
    right_motor -> setSpeed(50);

    left_motor -> run(BACKWARD);
    right_motor -> run(FORWARD);

    Serial.println("right");

}else if (vlls&&!vmlls&&!vmrls&&!vrls){

    Serial.println("junction to the left");

}else if (!vlls&&!vmlls&&!vmrls&&vrls){

    Serial.println("junction to the right");
    
}else{
    Serial.println("unidetnified situation");
}

return;

}
