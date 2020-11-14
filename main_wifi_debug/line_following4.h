int vlls, vmlls, vmrls, vrls;
WiFiUDP Udp;
IPAddress remoteip;
int remoteport;

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

    left_motor -> run(FORWARD);
    right_motor -> run(FORWARD);

    Serial.println("straight");
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("going straight");
    Udp.endPacket();
}
//T junction
else if (vlls&&vmlls&&vmrls&&vrls){

    left_motor -> setSpeed(150);
    right_motor -> setSpeed(150);

    left_motor -> run(FORWARD);
    right_motor -> run(FORWARD);

    Serial.println("T junction");
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("T junction");
    Udp.endPacket();
}

else if (!vlls&&!vmlls&&vmrls&&!vrls){

    left_motor -> setSpeed(100);
    right_motor -> setSpeed(50);

    left_motor -> run(FORWARD);
    right_motor -> run(FORWARD);

    Serial.println("slightly left");
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("going slightly left");
    Udp.endPacket();
}
else if (!vlls&&vmlls&&!vmrls&&!vrls){

    left_motor -> setSpeed(50);
    right_motor -> setSpeed(100);

    left_motor -> run(FORWARD);
    right_motor -> run(FORWARD);

    Serial.println("slightly right");
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("going slightly right");
    Udp.endPacket();
    
} else if (!vlls&&!vmlls&&vmrls&&vrls){

    left_motor -> setSpeed(50);
    right_motor -> setSpeed(50);

    left_motor -> run(BACKWARD);
    right_motor -> run(FORWARD);

    Serial.println("left");
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("going left");
    Udp.endPacket();

} else if (vlls&&vmlls&&!vmrls&&!vrls){

    left_motor -> setSpeed(50);
    right_motor -> setSpeed(50);

    left_motor -> run(FORWARD);
    right_motor -> run(BACKWARD);

    Serial.println("right");
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("going right");
    Udp.endPacket();

}else if (vlls&&!vmlls&&!vmrls&&!vrls){

    Serial.println("junction to the left");
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("junction to the left");
    Udp.endPacket();

}else if (!vlls&&!vmlls&&!vmrls&&vrls){

    Serial.println("junction to the right");
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("junction to the right");
    Udp.endPacket();
    
}else{
    Serial.println("unidetnified situation");
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("I am lost :(");
    Udp.endPacket();
}

return;

}
