#include <Adafruit_MotorShield.h>
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
//char user[] = SECRET_USER;       // your WPA2 enterprise username
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

int packetSize=0;
char packetBuffer[256]; //buffer to hold incoming packet
char ReplyBuffer[] = "acknowledged";       // a string to send back
char DataBuffer[52];
char DebugBuffer[12];
char EventBuffer[12];

WiFiUDP Udp;

IPAddress remoteip;
unsigned int remoteport=2390;

//===============END_WIFI===========

//pins
const int lineb= 0;
const int liner=1;
const int btn=2; //start/stop button
const int lls = 3; //line sensors left
const int mlls = 4; //middle lef
const int mrls = 5; //middle right
const int rls = 6; //right
const int ldr =7; //ldr interrput
const int blockled=8;
const int echoB=11;//detect ultrasonic
const int trigB=12;
const int going = 13;// flashing amber
const int blockc=A0;//block color
const int linec=A1;//line color
const int lr = A2; //red block transport
const int lb = A3; //blue block transport

Servo grabber;  // create a servo object

long duration; // variable for the duration of sound wave travel
int distance=999;

const int release_angle=60;
const int grab_angle=160;   // variable to hold the angle for the servo motor

bool running=false;
bool linechange=false;
//variables for sensors
int vlls, vmlls, vmrls, vrls;//line sensors
int vblock, vline;//ldrs

String linecolor="BLACK";

bool islinedetected=false;
bool islineevent=false;
bool iswelldefined=true;

const int vmax=90;//speed %
int v=vmax;

//cases b, r in pickup order 
const int bbrr[]={1,1,3,9,2,2,5,2,3,7,3,4,3,1,9,2,1,2,5,2,8,4,2,9,2,4,6,1,9,6,3,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//event1scare replaced todo
const int brbr[]={1,1,3,9,2,2,5,2,3,7,3,4,3,1,9,6,1,5,9,2,3,5,2,8,4,2,9,6,3,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const int brrb[]={1,1,3,9,2,2,5,2,3,7,3,4,3,1,9,6,1,5,9,2,4,6,5,9,2,3,5,2,8,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const int rrbb[]={1,1,3,9,2,2,4,10,1,3,1,9,6,1,5,9,2,3,5,2,3,7,3,3,9,2,1,3,6,5,9,2,3,5,2,8,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const int rbrb[]={1,1,3,9,2,2,4,10,1,3,1,9,2,1,2,5,2,3,7,3,3,9,2,1,3,1,6,1,5,9,2,4,6,5,9,2,3,5,2,8,3,0,0,0,0,0,0,0,0,0,0};//42
const int rbbr[]={1,1,3,9,2,2,4,10,1,3,1,9,2,1,2,5,2,3,7,3,3,9,2,1,3,1,6,1,5,9,2,3,5,2,8,4,2,9,6,3,4,0,0,0,0,0,0,0,0,0,0};

//int eventlist[]={2,8,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//testing
int eventlist[]={1,1,3,9,2,2,5,2,3,7,3,4,3,1,9,2,1,2,5,2,8,4,2,9,2,4,6,1,9,6,3,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};// new bbrr case (lots of 0 to fit everyting in)
int event=eventlist[0];
int eventcount=0;
unsigned long eventstart=0;
unsigned long time=0;
const unsigned long eventtimeout=2000;//TODO adjust this value

int blocknumber=0;
String block="";
String blockcolor[]={"","","",""};
String expected_blockcolor[]={"B","B","R","R"};
int extrared=-1;//variable for special red delivery

void blink();
void line();
void linedetect();
void reset();
void nextevent();
void start_line_event();
void end_line_event();
void all_line_detect();
void red_line_detect();
void blue_line_detect();
void block_color_detect();
void block_detect();
void forward();
void sleft();
void sright();
void left();
void right();
void stay();
void reverse();
void line_following();
void grab_routine();
void fill_eventlist(int*);
void update_eventlist();
void deposit_red();
void deposit_blue();
void special_deposit_blue();
void printWifiStatus();
void receive_routine(String);
void send_routine();

void setup() {//=======================SETUP==========================================================================================
  // put your setup code here, to run once:
  grabber.attach(9);

  AFMS.begin();

  Serial.begin(9600);
  pinMode(lls, INPUT_PULLUP);
  pinMode(mlls, INPUT_PULLUP);//INPUT PULLUP/DOWN?
  pinMode(mrls, INPUT_PULLUP);
  pinMode(rls, INPUT_PULLUP);
  pinMode(btn, INPUT_PULLUP);
  pinMode(ldr, INPUT);
  attachInterrupt(digitalPinToInterrupt(btn), blink, RISING);
  attachInterrupt(digitalPinToInterrupt(lls), line, CHANGE);
  attachInterrupt(digitalPinToInterrupt(mlls), line, CHANGE);
  attachInterrupt(digitalPinToInterrupt(mrls), line, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rls), line, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ldr), linedetect, RISING);
  pinMode(blockled, OUTPUT);
  pinMode(going, OUTPUT);
  pinMode(lr, OUTPUT);
  pinMode(lb, OUTPUT); 
  pinMode(liner, OUTPUT);
  pinMode(lineb, OUTPUT); 
  pinMode(trigB, OUTPUT); 
  pinMode(echoB, INPUT);

  Serial.begin(9600);   //initialize serial communication
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
    status = WiFi.begin(ssid, pass); //use this for phone
    //status = WiFi.beginEnterprise(ssid, user, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

  //WAIT FOR ONE MESSAGE TO DETERMINE IP

  // if there's data available, read a packet
  packetSize=0;
  do{
  packetSize = Udp.parsePacket();
  } while(!packetSize);

  
  Serial.print("Received packet of size ");
  Serial.println(packetSize);
  Serial.print("From ");
  remoteip=Udp.remoteIP();
  Serial.print(remoteip);
  Serial.print(", port ");
  Serial.println(Udp.remotePort());

  packetSize=0;
  // read the packet into packetBuffer
  int len = Udp.read(packetBuffer, 255);
  if (len > 0) {
    packetBuffer[len] = 0;
  }
  Serial.println("Contents:");
  Serial.println(packetBuffer);
  
  // send a reply, to the IP address and port that sent us the packet we received

  Udp.beginPacket(remoteip, remoteport);
  Udp.write("starting...");
  Udp.endPacket();

  Serial.println("setup finished");
}


void loop() {//=========================================================================================LOOP==================================
  // put your main code here, to run repeatedly:
  if(!running){
      Serial.println("not running");
      reset();
      delay(1000);
  }
  
  if(running){
    
    digitalWrite(going, HIGH); 

    switch(event){
      case 2:
        if(islineevent){//check line, in case interrupt doesnt work
          time=millis();
          if(time-eventstart>2000){//wait for leaving the line
            all_line_detect();
          }
        }
        if(linecolor=="WHITE"){
          linecolor="BLACK";
          linechange=true;
          islinedetected=true;
          Udp.beginPacket(remoteip, remoteport);
          Udp.write("linedetect function");
          Udp.endPacket();
        }
        break;
      case 3:
        if(islineevent){//check line, in case interrupt doesnt work
          time=millis();
          if(time-eventstart>2000){//wait for leaving the line
            all_line_detect();
          }
        }
        if(linecolor=="WHITE"){
          linecolor="BLACK";
          linechange=true;
          islinedetected=true;
          Udp.beginPacket(remoteip, remoteport);
          Udp.write("linedetect function");
          Udp.endPacket();
        }
        break;
      case 6:
        v=70;
        send_routine();
        deposit_red();
        break;
      case 7:
        v=70;
        send_routine();
        deposit_blue();
        break;
      case 8:
        v=70;
        send_routine();
        special_deposit_blue();
        break;
      case 9:        
        block_detect();
        Serial.print("ultrasonic distance: ");
        Serial.println(distance);
        if(distance<20){//slow down
          v=50;
        }
        if (distance<5){//test grabbing distance
          grab_routine();
          //reset
          v=vmax;
          distance=999;
        }
        break;
    }
    
    if(linechange){//at least one of the sensors had an interrutp
      linechange = false;
      
      //read sensor values
      vlls=digitalRead(lls);
      vmlls=digitalRead(mlls);
      vmrls=digitalRead(mrls);
      vrls=digitalRead(rls);
      
      sprintf(&DebugBuffer[0],"%06s","READ: ");
      sprintf(&DebugBuffer[6],"%01i",vlls);
      sprintf(&DebugBuffer[7],"%01i",vmlls);
      sprintf(&DebugBuffer[8],"%01i",vmrls);
      sprintf(&DebugBuffer[9],"%01i",vrls);
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(DebugBuffer);
      Udp.endPacket();
      Serial.println(DebugBuffer);

      //check if its simple line following
      iswelldefined=((vlls&&!vmlls&&vmrls&&vrls)||(vlls&&vmlls&&!vmrls&&vrls)||(!vlls&&!vmlls&&vmrls&&vrls)||(vlls&&vmlls&&!vmrls&&!vrls)||(vlls&&vmlls&&vmrls&&vrls));
      
      if (!iswelldefined&&event==10){//first red deposit
          Serial.println("event10routine");
          reverse();
          delay(1500);//reverse back from junction to allow easier grabbing test
          grabber.write(release_angle);
          digitalWrite(lr, LOW);
          delay(15);
          reverse();
          delay(1000);
          linechange = true;
          event=2;//start case 2
          start_line_event();
             
      }else if(event>5){//iswelldefined always false
        line_following();
        
      }else{
        //check timeout
        if(islineevent){  
          if (event==1||event==4 ||event ==5){//evetns that need timeout
            time=millis();
            if(time-eventstart>eventtimeout){//time's up
              end_line_event();
            }
          }
        }
        
        //set isevent if wasn't and if there is a not well defined reading eg not forward or turn
        if(!islineevent&&!iswelldefined){ 
          start_line_event();
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("event starting...");
            Udp.endPacket();

        }
        //set isevent if wasn't and if there is a not well defined reading eg not forward or turn
        if(islineevent){ // 0 1 2 3 4 5 are line events
          sprintf(&DebugBuffer[0],"%07s","EVENT: ");
          sprintf(&DebugBuffer[6],"%01i",event);
          sprintf(&DebugBuffer[7],"%02i",eventcount);
          Udp.beginPacket(remoteip, remoteport);
          Udp.write(DebugBuffer);
          Udp.endPacket();
          Serial.println(DebugBuffer);
          switch (event){            
            case 0://test, no events, only wifi can end this
              forward();
              delay(1000);
              stay();
              running=false;
              break;
            case 1: //keep going forward
              if(!iswelldefined){
                forward();
              } else {
                line_following();
              }
              break;
            case 2: //sharp left turn == turn until line detected
  
              if(!islinedetected){//==================watchout for misleading reading at start====================
                left();
              } else {//might call case 1 instead
                islinedetected=false;
                //special cases around blue deposits
                if(eventlist[eventcount+1]==8){
                  end_line_event();

                  left_motor -> setSpeed(1*v);
                  right_motor -> setSpeed(2*v);
  
                  left_motor -> run(FORWARD);
                  right_motor -> run(FORWARD);
                  delay(300);

                  linechange=true;
                }else if(eventlist[eventcount+2]==7){
                  end_line_event();
                  left_motor -> setSpeed(1*v);
                  right_motor -> setSpeed(2*v);
  
                  left_motor -> run(FORWARD);
                  right_motor -> run(FORWARD);
                  delay(300);
                  linechange=true;
                }else{

                  event=4;
                  linechange=true;
                  start_line_event();
                }
              }
              v=vmax;
              break;
            case 3: //sharp right turn == turn until line detected
              if(!islinedetected){                
                right();
              } else{
                 islinedetected=false;
                 //special cases around blue deposits
                 if(eventlist[eventcount+1]==7){
              
                  end_line_event();
                  left_motor -> setSpeed(2.5*v);
                  right_motor -> setSpeed(1*v);
  
                  left_motor -> run(FORWARD);
                  right_motor -> run(FORWARD);
                  delay(350);
                  linechange=true;
                } else if(eventlist[eventcount]==7){
                  //end_line_event();
                  left_motor -> setSpeed(2.5*v);
                  right_motor -> setSpeed(1*v);
  
                  left_motor -> run(FORWARD);
                  right_motor -> run(FORWARD);
                  delay(300);
                  event=5;
                  //islinedetected=false;
                  linechange=true;
                  start_line_event();

                }else{
                  event=5;
                  linechange=true;
                  start_line_event();
                }
              }
              v=vmax;
              break;
            case 4: //bended turn left == ignore right sensors
              vrls=HIGH; //no sharp right turn
              if(!iswelldefined){
                vmrls=HIGH;
                vrls=HIGH;
                line_following();
              }else {
                line_following();
              }
              break;
            case 5: //bended turn right == ignore left sensors
              vlls=HIGH; //no sharp left turn
              //special case when going from blue deposit to start
              if(extrared==blocknumber&&!digitalRead(lr)||eventlist[eventcount+4]==0){
                vlls=LOW;
              }
              if(!iswelldefined){
                vlls=HIGH;
                vmlls=HIGH;
                line_following();
              }else {
                line_following();
              }
              break;
          }
        } else {//well defined, no event
          line_following();
        }
        Serial.println("end linechange==========================================================");
      }
    }

  }

  //receive message through wifi
  packetSize = Udp.parsePacket();
  if (packetSize) {
    
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    //remoteip=Udp.remoteIP();
    //remoteport=Udp.remotePort();
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
    packetSize=0;
    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(remoteip, remoteport);
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }

}    //============================================================================END_LOOP========================================
///////INTERRUPT FUNCTIONS

void blink() {//switch on and off
  running = !running;
  linechange=true;
}
void line() {//line sensor interrupt
  linechange = true;
}

void linedetect(){//line color sensor interrupt
  //detect line using LDR
  if(islineevent&&(event==2||event==3)){//do something only at sharp turns
    time=millis();
    if(time-eventstart>1000){//wait for leaving the line
      linechange=true;
      islinedetected=true;
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("linedetect interrupt");
      Udp.endPacket();
    }
  }
}

void reset(){//reset everything
    Serial.println("reset");
    left_motor -> run(RELEASE);
    right_motor -> run(RELEASE);    
    grabber.write(release_angle);
    digitalWrite(going, LOW);
    digitalWrite(lr, LOW);
    digitalWrite(lb, LOW);
    digitalWrite(liner, LOW);
    digitalWrite(lineb, LOW);
    digitalWrite(blockled, LOW);
    distance=999;
    running=false;
    linechange=false;
    linecolor="BLACK";
    islinedetected=false;
    islineevent=false;
    iswelldefined=true;
    v=vmax;//speed %    
    fill_eventlist(bbrr);
    event=eventlist[0];
    eventcount=0;
    eventstart=0;
    time=0;
    blocknumber=0;
    extrared=-1;
    blockcolor[0]="";
    blockcolor[1]="";
    blockcolor[2]="";
    blockcolor[3]="";
    expected_blockcolor[0]="B";
    expected_blockcolor[1]="B";
    expected_blockcolor[2]="R";
    expected_blockcolor[3]="R";
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("reset");
    Udp.endPacket();
}

////////EVENTS
void nextevent(){//switch to the next evnet ni evnetlist, reset variables to prevent errors and make testing easier
  Serial.print("nextevent");
  Serial.println(eventcount);
  eventcount++;
  Serial.print("nextevent");
  Serial.println(eventcount);
  
  if (eventcount<(sizeof(eventlist)/sizeof(eventlist[0]))){
    event=eventlist[eventcount];
  } else {
    //running=false;
    eventcount=0;//enables restart from beginning
  }
  
  if(event==2||event==3){//preset LEDs to prevent unwanted interrupts
    digitalWrite(liner,HIGH);
    //digitalWrite(lineb, HIGH);
  } else if (event==7||event==8){
    digitalWrite(lineb, HIGH);
  }else{
    digitalWrite(liner,LOW);
    digitalWrite(lineb, LOW);
  }
  //might need delay
  islinedetected=false;//prevent unwanted interrupts
  
  sprintf(&EventBuffer[0],"%05s","NEXTE");
  sprintf(&EventBuffer[5],"%01i",event);
  sprintf(&EventBuffer[6],"%02i",eventcount);
  Udp.beginPacket(remoteip, remoteport);
  Udp.write(EventBuffer);
  Udp.endPacket();
  Serial.println(EventBuffer);
}

void start_line_event(){
  islineevent=true;
  eventstart=millis();
  if(event==2||event==3){//go forward to have the axle aligned with the line
    digitalWrite(liner, HIGH); 
    if(!(eventlist[eventcount]==7||eventlist[eventcount]==6||eventlist[eventcount]==10||eventlist[eventcount-1]==9)){
      forward();
      if(eventlist[eventcount+1]==7){
        delay(250);
      } else {
        delay(150);
      }
    }
  }
}

void end_line_event(){
  islineevent=false;
  nextevent();
}
///////sensors

void all_line_detect(){
  digitalWrite(liner, HIGH); //red light
  //digitalWrite(lineb,HIGH);
  if (analogRead(linec)<400){ //not black
     digitalWrite(liner, LOW); 
     //digitalWrite(lineb,LOW);
     linecolor="WHITE";
 } else{ 
     linecolor="BLACK";
 }  
}

void red_line_detect(){  
   digitalWrite(liner, HIGH); //red light
   delay(100);
   if (analogRead(linec)<300){ //red or white, try with blue light (600)
      digitalWrite(liner, LOW); 
      digitalWrite(lineb,HIGH);
      delay(100);
      if (analogRead(linec)<300){//(550)
        linecolor="WHITE";
      } else{
        linecolor="RED";
        stay();
        Udp.beginPacket(remoteip, remoteport);
        Udp.write("red line detected");
        Udp.endPacket();
       Serial.println("red line detected");
      }
      digitalWrite(lineb,LOW);
   }
   else{
      digitalWrite(liner, LOW); 
      linecolor="BLACK"; //or BLUE
   }  
}
void blue_line_detect(){
  digitalWrite(lineb,HIGH);
  //blue led should be already on
  if (analogRead(linec)<350){//(550)
      linecolor="BLUE"; //or white in case of an error
      stay();
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("blue line detected");
      Serial.println("blue line detected");
      Udp.endPacket();
      digitalWrite(lineb, LOW);
  } else{
      linecolor="BLACK";//or red but it doesn't matter
  }  
}

void block_color_detect(){
  digitalWrite(blockled, HIGH);
  delay(200);
  if(analogRead(blockc)<230){ //TODO find sensor values corresponding to each color
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("red block detected");
      Udp.endPacket();
      Serial.println("red block detected");
      block="R";
  } else{
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("blue block detected");
      Udp.endPacket();
      Serial.println("blue block detected");
      block="B";
  }
  digitalWrite(blockled, LOW);
}

void block_detect(){//ultrasonic read
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
}
/////////////////////////MOVES
void forward(){
      left_motor -> setSpeed(2.5*v);
      right_motor -> setSpeed(2.5*v);
  
      left_motor -> run(FORWARD);
      right_motor -> run(FORWARD);
      
      sprintf(&DebugBuffer[0],"%06s","STRA: ");
      sprintf(&DebugBuffer[6],"%01i",vlls);
      sprintf(&DebugBuffer[7],"%01i",vmlls);
      sprintf(&DebugBuffer[8],"%01i",vmrls);
      sprintf(&DebugBuffer[9],"%01i",vrls);
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(DebugBuffer);
      Udp.endPacket();
      Serial.println(DebugBuffer);
}
void sleft(){
      left_motor -> setSpeed(1.5*v);
      right_motor -> setSpeed(2*v);
  
      left_motor -> run(FORWARD);
      right_motor -> run(FORWARD);
  
      sprintf(&DebugBuffer[0],"%06s","SLEF: ");
      sprintf(&DebugBuffer[6],"%01i",vlls);
      sprintf(&DebugBuffer[7],"%01i",vmlls);
      sprintf(&DebugBuffer[8],"%01i",vmrls);
      sprintf(&DebugBuffer[9],"%01i",vrls);
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(DebugBuffer);
      Udp.endPacket();
      Serial.println(DebugBuffer);
}
void sright(){
      left_motor -> setSpeed(2*v);
      right_motor -> setSpeed(1.5*v);
  
      left_motor -> run(FORWARD);
      right_motor -> run(FORWARD);
  
      sprintf(&DebugBuffer[0],"%06s","SRIG: ");
      sprintf(&DebugBuffer[6],"%01i",vlls);
      sprintf(&DebugBuffer[7],"%01i",vmlls);
      sprintf(&DebugBuffer[8],"%01i",vmrls);
      sprintf(&DebugBuffer[9],"%01i",vrls);
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(DebugBuffer);
      Udp.endPacket();
      Serial.println(DebugBuffer);
}
void left(){
      left_motor -> setSpeed(v);
      right_motor -> setSpeed(v);
  
      left_motor -> run(BACKWARD);
      right_motor -> run(FORWARD);
  
      sprintf(&DebugBuffer[0],"%06s","LEFT: ");
      sprintf(&DebugBuffer[6],"%01i",vlls);
      sprintf(&DebugBuffer[7],"%01i",vmlls);
      sprintf(&DebugBuffer[8],"%01i",vmrls);
      sprintf(&DebugBuffer[9],"%01i",vrls);
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(DebugBuffer);
      Udp.endPacket();
      Serial.println(DebugBuffer);  
}
void right(){
      left_motor -> setSpeed(v);
      right_motor -> setSpeed(v);
  
      left_motor -> run(FORWARD);
      right_motor -> run(BACKWARD);

      sprintf(&DebugBuffer[0],"%06s","RIGH: ");
      sprintf(&DebugBuffer[6],"%01i",vlls);
      sprintf(&DebugBuffer[7],"%01i",vmlls);
      sprintf(&DebugBuffer[8],"%01i",vmrls);
      sprintf(&DebugBuffer[9],"%01i",vrls);
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(DebugBuffer);
      Udp.endPacket();
      Serial.println(DebugBuffer);
}
void stay(){
      left_motor -> run(RELEASE);
      right_motor -> run(RELEASE);
      
      sprintf(&DebugBuffer[0],"%06s","STOP: ");
      sprintf(&DebugBuffer[6],"%01i",vlls);
      sprintf(&DebugBuffer[7],"%01i",vmlls);
      sprintf(&DebugBuffer[8],"%01i",vmrls);
      sprintf(&DebugBuffer[9],"%01i",vrls);
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(DebugBuffer);
      Udp.endPacket();
      Serial.println(DebugBuffer);
}
void reverse(){
      left_motor -> setSpeed(150);
      right_motor -> setSpeed(150);
  
      left_motor -> run(BACKWARD);
      right_motor -> run(BACKWARD);
  
      sprintf(&DebugBuffer[0],"%06s","REVE: ");
      sprintf(&DebugBuffer[6],"%01i",vlls);
      sprintf(&DebugBuffer[7],"%01i",vmlls);
      sprintf(&DebugBuffer[8],"%01i",vmrls);
      sprintf(&DebugBuffer[9],"%01i",vrls);
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(DebugBuffer);
      Udp.endPacket();
      Serial.println(DebugBuffer);
}
void line_following(){ 
  if (vlls&&vmlls&&vmrls&&vrls){//1111
      forward();
  } else if (vlls&&!vmlls&&vmrls&&vrls){//1011
      sleft();
  } else if (vlls&&vmlls&&!vmrls&&vrls){//1101
      sright();
  } else if (!vlls&&!vmlls&&vmrls&&vrls){//0011
      left();
  } else if (vlls&&vmlls&&!vmrls&&!vrls){//1100
      right();
  }else{
      sprintf(&DebugBuffer[0],"%06s","ERROR:");//Shouldn't receive this
      sprintf(&DebugBuffer[6],"%01i",vlls);
      sprintf(&DebugBuffer[7],"%01i",vmlls);
      sprintf(&DebugBuffer[8],"%01i",vmrls);
      sprintf(&DebugBuffer[9],"%01i",vrls);
      Udp.beginPacket(remoteip, remoteport);
      Udp.write(DebugBuffer);
      Udp.endPacket();
      Serial.println(DebugBuffer);
    if(event==4){//for safety 0111 case only
      left_motor -> setSpeed(1*v);
      right_motor -> setSpeed(2*v);
  
      left_motor -> run(FORWARD);
      right_motor -> run(FORWARD);
    } else if(event==5){//for safety 1110 case only
      left_motor -> setSpeed(2*v);
      right_motor -> setSpeed(1*v);
  
      left_motor -> run(FORWARD);
      right_motor -> run(FORWARD);
    } else if(event==6){//backup plan for missing red line
      reverse();
      delay(500);
      v=vmax;
      grabber.write(release_angle);
      digitalWrite(lr, LOW);
      delay(15);
      reverse();
      delay(1000);//=====================================TEST===================
      linechange = true;
      event=2;//start case 2
      start_line_event();
    }else {//do something to recover
      left_motor -> setSpeed(50);
      right_motor -> setSpeed(50);
  
      left_motor -> run(FORWARD);
      right_motor -> run(FORWARD);      
    }
  }
}

void grab_routine(){
  Serial.println("grabroutine");
  stay();
  grabber.write(grab_angle); 
  send_routine();
  delay(1000);
  block_color_detect();
  if (block=="R"){
    digitalWrite(lr, HIGH);
    
  }else if (block=="B"){
    digitalWrite(lb, HIGH);

  } else { //ERROR=====
    digitalWrite(lr, HIGH);
    digitalWrite(lb, HIGH);
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("blockcolor error");
    Udp.endPacket();
  }
  if(blocknumber!=extrared){//extrared was transported to teh start, no update needed
    blockcolor[blocknumber]=block;
    update_eventlist();
  }
  block="";
  nextevent();
  linechange = true;
  if(event==2){
    start_line_event();
  }
}

void fill_eventlist(int* events){
   int i;
   
   for (i=0; i<46;i++){ 
      eventlist[i]=events[i];
      //Serial.print(events[i]);
      Serial.print(eventlist[i]);
      Serial.print(",");
   }
   Serial.println("end");
}

void update_eventlist(){
  if(blocknumber!=extrared){
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("update eventlist");
    Udp.endPacket();
    blocknumber++;//block being transported
    if(expected_blockcolor[blocknumber-1]!=blockcolor[blocknumber-1]){//eventlist and expected color has to be changed
      switch(blocknumber){
        case 1:
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("update case1");
            Udp.endPacket();
          if(blockcolor[0]=="R"){ 
            extrared=3;
            fill_eventlist(rrbb);
            expected_blockcolor[0]="R";
            expected_blockcolor[1]="R";
            expected_blockcolor[2]="B";
            expected_blockcolor[3]="B";
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("eventlist filled with rrbb");
            Udp.endPacket();
          } else {
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("error1");
            Udp.endPacket();
          }
          break;
        case 2:
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("update case2");
            Udp.endPacket();
          if(blockcolor[1]=="B"&&blockcolor[0]=="R"){//RB
            extrared=2;
            fill_eventlist(rbrb);
            expected_blockcolor[0]="R";
            expected_blockcolor[1]="B";
            expected_blockcolor[2]="R";
            expected_blockcolor[3]="B";
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("eventlist filled with rbrb");
            Udp.endPacket();
          } else if(blockcolor[1]=="R"&&blockcolor[0]=="B"){//BR
            fill_eventlist(brbr);
            expected_blockcolor[0]="B";
            expected_blockcolor[1]="R";
            expected_blockcolor[2]="B";
            expected_blockcolor[3]="R";
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("eventlist filled with brbr");
            Udp.endPacket();
          } else {
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("error2");
            Udp.endPacket();
          }
          break;
        case 3:
          if(blockcolor[2]=="B"&&blockcolor[1]=="B"&&blockcolor[0]=="R"){//RBBR
            fill_eventlist(rbbr);
            expected_blockcolor[0]="R";
            expected_blockcolor[1]="B";
            expected_blockcolor[2]="B";
            expected_blockcolor[3]="R";
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("eventlist filled with rbbr");
            Udp.endPacket();
          } else if(blockcolor[2]=="R"&&blockcolor[1]=="R"&&blockcolor[0]=="B"){//BRRB
            fill_eventlist(brrb);
            expected_blockcolor[0]="B";
            expected_blockcolor[1]="R";
            expected_blockcolor[2]="R";
            expected_blockcolor[3]="B";
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("eventlist filled with brrb");
            Udp.endPacket();
          } else {
            Udp.beginPacket(remoteip, remoteport);
            Udp.write("error3");
            Udp.endPacket();
          }
          break;
      }
    }
  } else{
    Udp.beginPacket(remoteip, remoteport);
    Udp.write("extrared reset");
    Udp.endPacket();
    extrared=-1;
  }
}

void deposit_red(){
  red_line_detect();//search for red line
  if(linecolor=="RED"){
    v=vmax;
    grabber.write(release_angle);
    digitalWrite(lr, LOW);
    delay(15);
    reverse();
    delay(1000);//=====================================TEST===================
    linechange = true;
    event=2;//start case 2
    start_line_event();
  }
}

void deposit_blue(){
  blue_line_detect();//search for blue line 
  if(linecolor=="BLUE"){
    v=vmax;
    grabber.write(release_angle);
    digitalWrite(lb, LOW);
    delay(15);
    reverse();
    while(digitalRead(rls)||digitalRead(mrls)||digitalRead(mlls)){} //reverse until right sensor detects line
    
    linechange = true;
    event=3; //start case 3
    start_line_event();
  }
}
void special_deposit_blue(){
  blue_line_detect();//search for blue line 
  if(linecolor=="BLUE"){
    v=vmax;
    grabber.write(release_angle);
    digitalWrite(lb, LOW);
    digitalWrite(lr, LOW);//special case
    delay(15);
    reverse();
    while(digitalRead(lls)){} //reverse until left sensor detects line
    delay(200);
    linechange = true;
    event=2; //start case 2
    start_line_event();
  }
}

//////////////WIFI FUNCTIONS
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
  if(line=="0"){
    event=0;
    islinedetected=false;
    linechange=true;
    start_line_event();

  } else if (line=="1"){
    event=1;
    islinedetected=false;
    linechange=true;
    start_line_event();

  } else if (line=="2"){
    event=2;
    islinedetected=false;
    linechange=true;
    start_line_event();

  } else if (line=="3"){
    event=3;
    islinedetected=false;
    linechange=true;
    start_line_event();

  } else if (line=="4"){
    event=4;
    islinedetected=false;
    linechange=true;
    start_line_event();

  } else if (line=="5"){
    event=5;
    islinedetected=false;
    linechange=true;
    start_line_event();
    
  } else if (line=="00"){
    event=0;
    islinedetected=false;
    linechange=true;

  } else if (line=="11"){
    event=1;
    islinedetected=false;
    linechange=true;

  } else if (line=="22"){
    event=2;
    islinedetected=false;
    linechange=true;

  } else if (line=="33"){
    event=3;
    islinedetected=false;
    linechange=true;

  } else if (line=="44"){
    event=4;
    islinedetected=false;
    linechange=true;

  } else if (line=="55"){
    event=5;
    islinedetected=false;
    linechange=true;
     
  } else if (line=="6"){
    event=6;
    islinedetected=false;
    linechange=true;

  } else if (line=="7"){
    event=7;
    islinedetected=false;
    linechange=true;

  } else if (line=="8"){
    event=8;
    islinedetected=false;
    linechange=true;
    
  } else if (line=="9"){
    event=9;
    islinedetected=false;
    linechange=true;
    
  } else if (line=="10"){
    event=10;
    islinedetected=false;
    linechange=true;
    
  } else if (line=="on"){
    blink();
    running=true;
    
  } else if (line=="off"){
    running=false; 
    
  } else if (line=="e"){//send eventlist
    int i;
    for (i=0; i<sizeof(eventlist)/sizeof(eventlist[0]);i++){
      sprintf(&DataBuffer[i],"%01i",eventlist[i]);
    }
    Udp.beginPacket(remoteip, remoteport);
    Udp.write(DataBuffer);
    Udp.endPacket();
    Serial.println(DataBuffer);
    //reset();
    
  } else if (line[0]=='e'){//fill event_list, restart
    //running=false;
    Serial.println("eventupdate received");
    int i;
    for (i=0; i<packetSize-1;i++){
      eventlist[i]=(int)line[i+1]-(int)"0";
    }    
    //reset();
    left_motor -> run(RELEASE);
    right_motor -> run(RELEASE);    
    grabber.write(release_angle);
    digitalWrite(going, LOW);
    digitalWrite(lr, LOW);
    digitalWrite(lb, LOW);
    digitalWrite(liner, LOW);
    digitalWrite(lineb, LOW);
    digitalWrite(blockled, LOW);
    distance=999;
    linechange=true;
    linecolor="BLACK";
    islinedetected=false;
    islineevent=false;
    iswelldefined=true;
    v=vmax;//speed %    
    event=eventlist[0];
    eventcount=0;
    eventstart=0;
    time=0;
    blocknumber=0;
    extrared=-1;
    blockcolor[0]="";
    blockcolor[1]="";
    blockcolor[2]="";
    blockcolor[3]="";
    expected_blockcolor[0]="B";
    expected_blockcolor[1]="B";
    expected_blockcolor[2]="R";
    expected_blockcolor[3]="R";
    
  } else if (line=="liner"){//lights
    digitalWrite(liner,!digitalRead(liner));
    
  } else if (line=="lineb"){//lights
    digitalWrite(lineb, !digitalRead(lineb));
  } else if (line=="blockled"){//lights
    digitalWrite(blockled, !digitalRead(blockled));
  } else if (line=="bd"){
    digitalWrite(lineb, HIGH);
    delay(100);
    blue_line_detect();
    if(linecolor=="BLUE"){
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("blueline");
      Udp.endPacket();
    }  else if(linecolor=="RED"){
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("red line");
      Udp.endPacket();
    }  else if(linecolor=="WHITE"){
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("white");
      Udp.endPacket();
    }  else if(linecolor=="BLACK"){
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("black");
      Udp.endPacket();
    }
  } else if (line=="rd"){
    red_line_detect();
    if(linecolor=="BLUE"){
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("blueline");
      Udp.endPacket();
    }  else if(linecolor=="RED"){
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("red line");
      Udp.endPacket();
    }  else if(linecolor=="WHITE"){
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("white");
      Udp.endPacket();
    }  else if(linecolor=="BLACK"){
      Udp.beginPacket(remoteip, remoteport);
      Udp.write("black");
      Udp.endPacket();
    }

  } else if (line=="f") { //forward
    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);
    left_motor -> run(FORWARD);
    right_motor -> run(FORWARD);
    Serial.println("forward");
    
  } else if (line=="s") { //stop
    left_motor -> run(RELEASE);
    right_motor -> run(RELEASE);
    
  } else if (line=="r") { //right
    left_motor -> setSpeed(50);
    right_motor -> setSpeed(50);
    left_motor -> run(FORWARD);
    right_motor -> run(BACKWARD);
    
  } else if (line=="l") { //left
    left_motor -> setSpeed(50);
    right_motor -> setSpeed(50);
    left_motor -> run(BACKWARD);
    right_motor -> run(FORWARD);
    
  }else if (line=="b") { //reverse
    left_motor -> setSpeed(100);
    right_motor -> setSpeed(100);
    left_motor -> run(BACKWARD);
    right_motor -> run(BACKWARD);
    
  }else if (line=="g") { //grab
    grabber.write(grab_angle);
    //delay(15);
  }else if (line=="d") { //deposit
    grabber.write(release_angle);
    //delay(15);
  }else if (line=="a") { //send sensordata
    send_routine();
  }
}

void send_routine(){
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
  vblock=analogRead(blockc);
  sprintf(&DataBuffer[19],"%04i",vblock);
  
  //colorline
  sprintf(&DataBuffer[23],"%09s","\nColorL: ");
  vline=analogRead(A1);
  sprintf(&DataBuffer[32],"%04i",vline);
    
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
  sprintf(&DataBuffer[46],"%04i",distance);
  
  Serial.println(DataBuffer);
  distance=999;
 
  Udp.beginPacket(remoteip, remoteport);
  Udp.write(DataBuffer);
  Udp.endPacket();
}
