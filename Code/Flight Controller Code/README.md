Folder for Arduino Flight Controller and associated files 

ESC_calibration Code
- This code is used to test the motor to make sure that they're fully connected and able to spin. Can use this code to test which direction the motors are rotating. 

Wifi_Test Code
- This code is used to test out the wifi connection of the board and check to see if there are any income packets through the UPD port








Flight Controller Code
- This is the flight controller for the drone that works with an IMU. It takes reading from IMU and uses PID control system to stabilise the drone during flight. 
The Board used for this FC is the Arduino UNO WiFi Rev.2.


Motor setup for the drone
                       1   2   
                         \ /    
                         / \    
                       3   4 
library needed to run the code doesn't come native on Arduino and so must be downloaded and added onto the 

#include <WiFiClient.h>  // https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/WiFiClient.h
#include <WiFiUdp.h>   // https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/WiFiUdp.h
#include <WiFiServer.h>  // https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/WiFiServer.h
#include <WiFiNINA.h>  // https://github.com/arduino-libraries/WiFiNINA

<WiFiNINA.h>
This wifi library is specific to the Arduino UNO WiFi Rev.2 WiFi  board. It can serve as ei
 can instantiate Servers, Clients and send/receive UDP packets through WiFi. The board can connect either to open or encrypted networks (WEP, WPA). The IP address can be assigned statically or through a DHCP. The library can also manage DNS
