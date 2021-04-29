
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiNINA.h>
#include <Wire.h>


#define PORT 5880         // UDP port
char ssid[] = "drone_controller";  //Enter SSID
char pass[] = "fly_high64"; //Enter Password

int          status;
WiFiUDP         Udp;

void setup() {
  Serial.begin(115200);  /******************************************/
  Serial.println("Starting set up");  // Connect to WiFi
   // Serial.println(WiFi.scanNetworks());

  while (WiFi.status() == WL_NO_MODULE)
   {
             Serial.println("no shield");
   }
  Serial.print("connecting to ");
  Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect();
    WiFi.begin(ssid, pass);
    delay(100);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
//listens to see if anything is being sent through the UDP port
  while (!Udp.begin(PORT))  {
    Serial.println("waiting for udp connection");
  }

  Serial.print("set up complete");

}

void loop() {
  // put your main code here, to run repeatedly:

}
