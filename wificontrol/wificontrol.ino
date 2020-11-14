#include<Adafruit_MotorShield.h>
#include <SPI.h>
#include <WiFiNINA.h>

#include "arduino_secrets.h" 

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *left_motor = AFMS.getMotor(1);
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);
//=======================WIFI========
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);
//===============END_WIFI===========
//test LEDs
const int going = 6;
const int lr = 7;
const int lb = 8;

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
  pinMode(going, OUTPUT);
  pinMode(lr, OUTPUT);
  pinMode(lb, OUTPUT);
  
//=======================WIFI==========================================================================================
  Serial.begin(9600);      // initialize serial communication
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
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status

  //===============END_WIFI=====================================================================================
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
    WiFiClient client = server.available();   // listen for incoming clients
  
    if (client) {                             // if you get a client,
      Serial.println("new client");           // print a message out the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          if (c == '\n') {                    // if the byte is a newline character
  
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              showWebPage(client);
              // break out of the while loop:
              break;
            } else {    // if you got a newline, then clear currentLine:
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
  
          // Check to see if the client request was "GET /H" or "GET /L":
          performRequest(currentLine);
        }
      }
      // close the connection:
      client.stop();
      Serial.println("client disonnected");
    }
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

void showWebPage(WiFiClient client) {
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a content-type so the client knows what's coming, then a blank line:
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println();

  // the content of the HTTP response follows the header:
  client.println("<h1>Arduino Remote Control</h1>");
  client.println("<table border=1 style='text-align:center'>");
  client.println("<tr><th>Component</th><th>Status</th><th>Control</th></tr>");

  // LEFT sensor
  client.print("<tr><td>Left sensor</td><td>");
  if (digitalRead(lls)) {
    client.print("<font style='color:red;'>OFF</font>");
  } else {
    client.print("<font style='color:green;'>ON</font>");
  }
  client.println("</td><td><a href='/Left sensor/off'>OFF</a> / <a href='/Left sensor/on'>ON</a></td></tr>");

  // MIDDLE sensor
  client.print("<tr><td>Middle sensor</td><td>");
  if (digitalRead(mls)) {
    client.print("<font style='color:red;'>OFF</font>");
  } else {
    client.print("<font style='color:green;'>ON</font>");
  }
  client.println("</td><td><a href='/Middle sensor/off'>OFF</a> / <a href='/Middle sensor/on'>ON</a></td></tr>");

  // RIGHT sensor
  client.print("<tr><td>Right sensor</td><td>");
  if (digitalRead(rls)) {
    client.print("<font style='color:red;'>OFF</font>");
  } else {
    client.print("<font style='color:green;'>ON</font>");
  }
  client.println("</td><td><a href='/Right sensor/off'>OFF</a> / <a href='/Right sensor/on'>ON</a></td></tr>");

  //CONTROL
  client.print("<tr><td><a href='/Forward'>FORWARD</a></td>");
  client.print("<td><a href='/Stop'>STOP</a></td>");
  client.print("<td><a href='/Right'>RIGHT</a></td>");
  client.println("<td><a href='/Left'>LEFT</a></td></tr>");

  client.println("</table>");

  // The HTTP response ends with another blank line
  client.println();
}

void performRequest(String line) {
  if(running){
    if (line.endsWith("GET /Forward")) {
      left_motor -> setSpeed(100);
      right_motor -> setSpeed(100);
      left_motor -> run(BACKWARD);
      right_motor -> run(BACKWARD);
      digitalWrite(lr, HIGH);
      digitalWrite(lb, HIGH);
      
    } else if (line.endsWith("GET /Stop")) {
      left_motor -> run(RELEASE);
      right_motor -> run(RELEASE);
      digitalWrite(lr, LOW);
      digitalWrite(lb, LOW);
      
    } else if (line.endsWith("GET /Right")) {
      left_motor -> setSpeed(150);
      right_motor -> setSpeed(150);
      left_motor -> run(FORWARD);
      right_motor -> run(BACKWARD);
      digitalWrite(lr, LOW);
      digitalWrite(lb, HIGH);
      
    } else if (line.endsWith("GET /Left")) {
      left_motor -> setSpeed(150);
      right_motor -> setSpeed(150);
      left_motor -> run(BACKWARD);
      right_motor -> run(FORWARD);
      digitalWrite(lr, HIGH);
      digitalWrite(lb, LOW);
    }
  }
}
