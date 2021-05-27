#include <Arduino_LSM6DS3.h>
#include "MadgwickAHRS.h"
//https://itp.nyu.edu/physcomp/lessons/accelerometers-gyros-and-imus-the-basics/
#include <SPI.h>
#include <WiFiNINA.h>

Madgwick filter;
const float sensorRate = 104.00;
float headingp= 180.0, rollp= 0.0, pitchp= 0.0;
char ssid[] = "WL520GC";
char pass[] = "82RsB6EAC1";
int keyIndex = 0;
int status = WL_IDLE_STATUS;
char server[] = "192.168.1.236";
WiFiClient client;

void setup() {
 pinMode(LED_BUILTIN, OUTPUT);
 Serial.begin(9600);
 //while (!Serial);
 if (!IMU.begin()) {
 Serial.println("Failed to initialize IMU!");
 while (1);
 }
 Serial.print("Gyroscope sample rate = ");
 Serial.print(IMU.gyroscopeSampleRate());
 Serial.println(" Hz");
 Serial.println();
 filter.begin(sensorRate);
 
 // check for the WiFi module:
 if (WiFi.status() == WL_NO_MODULE) {
   Serial.println("Communication with WiFi module failed!");
   // don't continue
   while (true);
 }
 // attempt to connect to Wifi network:
 while (status != WL_CONNECTED) {
   Serial.print("Attempting to connect to SSID: ");
   Serial.println(ssid);
   status = WiFi.begin(ssid, pass);
   // wait 10 seconds for connection:
   delay(10000);
 }
 Serial.println("Connected to wifi");
 printWifiStatus();
 Serial.println("READY!");
 digitalWrite(LED_BUILTIN, HIGH); 
}
void loop() {
 float x, y, z, xc, yc, zc;
 float xA, yA, zA;
 float xAc, yAc, zAc;
 float heading, roll, pitch;
 if (IMU.gyroscopeAvailable() && IMU.gyroscopeAvailable()) {
  IMU.readGyroscope(x, y, z);
  IMU.readAcceleration(xA, yA, zA);
  if(x>=0) xc = floor(x); else xc =-floor(-x);
  if(y>=0) yc = 10.0*floor(y*0.1); else yc=-10.0*floor(-y*0.1);
  if(z>=0) zc = 4.0*floor(z*0.25); else zc=-4.0*floor(-z*0.25);
  if(xA>=0) xAc = floor(xA); else xAc =-floor(-xA);
  if(yA>=0) yAc = 0.1*floor(yA*10.0); else yAc =-0.1*floor(-yA*10.0);
  if(zA>=0) zAc = 4.0*floor(zA*0.25); else zAc=-4.0*floor(-zA*0.25);
  filter.updateIMU(xc, yc, zc, xAc, yAc, zAc);
  heading = filter.getYaw();
  roll = filter.getRoll();
  pitch = filter.getPitch();
  if(fabs(heading - headingp)>15.0 || abs(roll - rollp)>15.0 || abs(pitch - pitchp)>15.0)
  {
    Serial.println(heading);
    headingp = heading; 
    pitchp= pitch;
    rollp = roll;
    Serial.println("\nStarting connection to server...");
    if (client.connect(server, 80)) 
    {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("connected to server");
      // Make a HTTP request:
      client.println("GET /SOS");
      client.println("Host: 192.168.1.236");
      client.println("Connection: close");
      client.println();
      while (client.available()) {
        char c = client.read();
        Serial.write(c);
       }
      // if the server's disconnected, stop the client:
      if (!client.connected()) {
        Serial.println();
        Serial.println("disconnecting from server.");
        client.stop();
        // do nothing forevermore:
        // while (true);
      }
    }
  }
 }
}
void printWifiStatus() {
 // print the SSID of the network you're attached to:
 Serial.print("SSID: ");
 Serial.println(WiFi.SSID());
 // print your WiFi shield's IP address:
 IPAddress ip = WiFi.localIP();
 Serial.print("IP Address: ");
 Serial.println(ip);
 // print the received signal strength:
 long rssi = WiFi.RSSI();
 Serial.print("signal strength (RSSI):");
 Serial.print(rssi);
 Serial.println(" dBm");
}
