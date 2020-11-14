#include<Adafruit_MotorShield.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Servo.h>

#include "arduino_secrets.h" 

int status = WL_IDLE_STATUS;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *left_motor = AFMS.getMotor(1);
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);
//=======================WIFI========
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[256]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back
char DataBuffer[50];
char ScanBuffer[256];//+(180/5+1)*4];




WiFiUDP Udp;

IPAddress remoteip;
int remoteport;

//===============END_WIFI===========
//test LEDs
const int going = 13;//flashing
const int lr = A2;
const int lb = A3; 

//pins
const int btn=2; //start/stop button
const int lls = 3; //line sensors
const int mlls = 4;
const int mrls = 5;
const int rls = 6;
const int echoS=7;//scan
const int trigS=8;
const int echoB=11;//detect
const int trigB=12;

//color sensing: A0-block, A1-line

Servo grabber;  // create a servo object
Servo scanner;

int pos = 0;//scanner

long duration; // variable for the duration of sound wave travel
int scandata[180/5+1];//scan every 5 degree

int distance;
int release_angle=40;//========================================check these values
int grab_angle=120;   // variable to hold the angle for the servo motor

bool running=true;//=================================================EDIT===================================================
//variables for sensors
int vlls, vmlls, vmrls, vrls, vblock, vline;


void setup() {
  // put your setup code here, to run once:
  grabber.attach(9);
  scanner.attach(10);
  
  AFMS.begin();

  Serial.begin(9600);
  pinMode(lls, INPUT);
  pinMode(mlls, INPUT);
  pinMode(mrls, INPUT);
  pinMode(rls, INPUT);
  pinMode(btn, INPUT);
  attachInterrupt(digitalPinToInterrupt(btn), blink, RISING);
  pinMode(going, OUTPUT);
  pinMode(lr, OUTPUT);
  pinMode(lb, OUTPUT);
  
//=======================WIFI==========================================================================================
  Serial.begin(9600);      // initialize serial communication
  while (!Serial) { //==================================================================================================================================MIGHT NO NEED FOR THIS WHILE
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  //pinMode(redLED, OUTPUT);
  //pinMode(yellowLED, OUTPUT);
  //pinMode(greenLED, OUTPUT);      // set the LED pin mode

    // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

  //===============END WIFI=====================================================================================
  //========================WAIT FOR STARTING PACKAGE=================== (start)--DOESN'T WORK, ONLY WAITS FOR ONE MESSAGE, BUT IT'S FINE
  while(packetBuffer!="start"){
    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      remoteip=Udp.remoteIP();
      remoteport=Udp.remotePort();
      Serial.print(remoteip);
      Serial.print(", port ");
      Serial.println(remoteport);
  
      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
      Serial.println("Contents:");
      Serial.println(packetBuffer);
      
      // send a reply, to the IP address and port that sent us the packet we received
      if (packetBuffer!=""){
        Udp.beginPacket(remoteip, remoteport);
        Udp.write("starting...");
        Udp.endPacket();
        break;
      }
    }
  }
}

void blink() {
  running = !running;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!running){
      left_motor -> run(RELEASE);
      right_motor -> run(RELEASE);
      digitalWrite(going, LOW);
      digitalWrite(lr, LOW);
      digitalWrite(lb, LOW);
      Serial.println("not running");
      delay(100);//TODO not quite good
  }
  
  while(running){
    digitalWrite(going, HIGH);
    //=======================WIFI====================================================================================================
    send_routine();
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      remoteip=Udp.remoteIP();
      remoteport=Udp.remotePort();
      Serial.print(remoteip);
      Serial.print(", port ");
      Serial.println(remoteport);
  
      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }

      Serial.println("Contents:");
      Serial.println(packetBuffer);
      receive_routine(packetBuffer);
      // send a reply, to the IP address and port that sent us the packet we received
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(ReplyBuffer);
      Udp.endPacket();
    }

    delay(200);
    //==============================END_WIFI======================================================================================
  }
  /*
  while(running){
    long start = millis();
    long end = start;

    while((end - start) < 1000){
      line_following(left_motor,right_motor,lls,mls,rls);
      end = millis();
    }
  }*/

}
//==============WIFI_FUNCTIONS================================================================================
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void receive_routine(String line) {
  
  if(running){
    if (line=="f") { //forward
      left_motor -> setSpeed(100);
      right_motor -> setSpeed(100);
      left_motor -> run(BACKWARD);
      right_motor -> run(BACKWARD);
      digitalWrite(lr, HIGH);
      digitalWrite(lb, HIGH);
       Serial.println("forward");
      
    } else if (line=="s") { //stop
      left_motor -> run(RELEASE);
      right_motor -> run(RELEASE);
      digitalWrite(lr, LOW);
      digitalWrite(lb, LOW);
      
    } else if (line=="r") { //right
      left_motor -> setSpeed(50);
      right_motor -> setSpeed(50);
      left_motor -> run(FORWARD);
      right_motor -> run(BACKWARD);
      digitalWrite(lr, LOW);
      digitalWrite(lb, HIGH);
      
    } else if (line=="l") { //left
      left_motor -> setSpeed(50);
      right_motor -> setSpeed(50);
      left_motor -> run(BACKWARD);
      right_motor -> run(FORWARD);
      digitalWrite(lr, HIGH);
      digitalWrite(lb, LOW);
      
    }else if (line=="b") { //reverse
      left_motor -> setSpeed(100);
      right_motor -> setSpeed(100);
      left_motor -> run(FORWARD);
      right_motor -> run(FORWARD);
      digitalWrite(lr, LOW);
      digitalWrite(lb, LOW);
    }else if (line=="grab") { //grab
      grabber.write(grab_angle);
      delay(15);
    }else if (line=="release") { //release
      grabber.write(release_angle);
      delay(15);
    }else if (line=="scan") { //scan
      sprintf(&ScanBuffer[0],"%09s","UltraSS: ");
      for (pos = 0; pos <= 180; pos += 5) { // goes from 0 degrees to 180 degrees
        // in steps of 5 degree
        scanner.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
        // Clears the trigPin condition
        digitalWrite(trigS, LOW);
        delayMicroseconds(2);
        // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
        digitalWrite(trigS, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigS, LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(echoS, HIGH);
        // Calculating the distance
        scandata[pos/5] = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
        // Displays the distance on the Serial Monitor
        Serial.print("Distance: ");
        Serial.print(scandata[pos/5]);
        Serial.println(" cm");
        
        sprintf(&ScanBuffer[9+pos/5*5],"%04i",scandata[pos/5]);
        
      }
      pos=0;
      scanner.write(pos);
      delay(15);
      Serial.println(ScanBuffer);
      //send results
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(ScanBuffer);
      Udp.endPacket();
    }
  }
}

void send_routine(void){
  Serial.print("sendroutine");
  //linesensors
  sprintf(&DataBuffer[0],"%06s","Line: ");
  
  vlls=digitalRead(lls);
  sprintf(&DataBuffer[6],"%01i",vlls);
  vmlls=digitalRead(mlls);
  sprintf(&DataBuffer[7],"%01i",vmlls);
  vmrls=digitalRead(mrls);
  sprintf(&DataBuffer[8],"%01i",vmrls);
  vrls=digitalRead(rls);
  sprintf(&DataBuffer[9],"%01i",vrls);

  //colorblock
  sprintf(&DataBuffer[10],"%09s","\nColorB: ");
  vblock=analogRead(A0);
  sprintf(&DataBuffer[19],"%04i",vblock);
  
  //colorline
  sprintf(&DataBuffer[23],"%09s","\nColorL: ");
  vline=analogRead(A1);
  sprintf(&DataBuffer[32],"%04i",vblock);
    
  //ultrasonic block
  // Clears the trigPin condition
  digitalWrite(trigB, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigB, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigB, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoB, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  //Serial.print("Block distance: ");
  //Serial.print(distance);
  //Serial.println(" cm");
  sprintf(&DataBuffer[36],"%10s","\nUltraSB: ");
  sprintf(&DataBuffer[46],"%04i",vblock);
  Serial.println(vblock);
  Serial.println(DataBuffer);

 
   Udp.beginPacket(remoteip, remoteport);
   Udp.write(DataBuffer);
   Udp.endPacket();
}
