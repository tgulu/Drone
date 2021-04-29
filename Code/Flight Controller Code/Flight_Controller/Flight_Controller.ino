#include <Wire.h> //The I2C library
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFi.h>
#include <SPI.h>
#include <Servo.h>

#define MPU6050 0x68 //Device address
#define ACCEL_CONFIG 0x1C //Accelerometer configuration address
#define GYRO_CONFIG 0x1B GYRO_CONFIG register (1B hex)

//Registers: Accelerometer, Gyroscope
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

//https://forum.arduino.cc/t/how-to-use-i2c-and-mpu6050/430974
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define acc_readings 6;
#define gyro_readings 6;

#define toRead 6
byte reading[toRead];

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN1 7
#define MOTOR_PIN2 8
#define MOTOR_PIN3 9
#define MOTOR_PIN4 10

#define BUFSIZE 25 
#define PORT 4791         // UDP port

int DELAY = 1000;
//Sensor output scaling
#define accSens 0 // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 0 // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s


Servo motor1;             /* Motors (servos), numbering:   1   2   */
Servo motor2;             /*                                \ /    */
Servo motor3;             /*                                / \    */
Servo motor4;             /*                               3   4   */


char ssid[] = "Drone_Wifi";     // WiFi
char pass[] = "flight_controller"; 
char rbuf[BUFSIZE]; // buffer to hold incoming packet,
char wbuf[BUFSIZE];
int          status;
WiFiUDP         Udp;

// Commanded values (from Android)
byte     cThrottle;     
short         cYaw;
short        cRoll;
short       cPitch;
boolean    cCutoff; 


void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

void readFrom(byte device, byte address, byte bytes, byte reading[]) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(device, bytes);
  while (bytes > Wire.available()){
  }
  for (int i = 0; i < bytes; i++){
    reading[i] = Wire.read();
  }
}

/*
https://arduino.stackexchange.com/questions/38908/how-can-arduino-know-wire-available-is-true-or-false
https://forum.arduino.cc/t/imu-doesnt-work-for-vertical-position/442502
 

//Function for writing a byte to an address on an I2C device
void writeTo(byte device, byte toAddress, byte val) {
  Wire.beginTransmission(device);  
  Wire.write(toAddress);        
  Wire.write(val);        
  Wire.endTransmission();
}

//Function for reading num bytes from addresses on an I2C device
void readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}
*/

//https://forum.arduino.cc/t/scaling-gyroscope-output-mpu6050/180436
//Function for reading the gyro.
void gyro() {
  readFrom(MPU6050, GYRO_XOUT_H, toRead, reading);
  float xgyro = ((reading[0] << 8) | reading[1]);
  //xgyro /= (float)(1<<(16-gyroSens))/250;
  float ygyro = ((reading[2] << 8) | reading[3]);
  // "Scale output somehow"
  float zgyro = ((reading[4] << 8) | reading[5]);

  float resultant = sqrt(pow(xgyro,2) +pow(ygyro,2) + pow(zgyro,2));

  Serial.print(xgyro,3);
  Serial.print('\t');
  Serial.print(ygyro,3);
  Serial.print('\t');
  Serial.print(zgyro,3);
  Serial.print('\t');
  Serial.println(resultant,3);
  delay(100);
} 

//Function for reading the accelerometer.
void acc() {
  readFrom(MPU6050, ACCEL_XOUT_H, toRead, reading);
  float xacc = ((reading[0] << 8) | reading[1]);
  xacc /= (float)(1<<(16-accSens-2));
  float yacc = ((reading[2] << 8) | reading[3]);
  yacc /= (float)(1<<(16-accSens-2));
  float zacc = ((reading[4] << 8) | reading[5]);
  zacc /= (float)(1<<(16-accSens-2));
  float resultant = sqrt(pow(xacc,2) +pow(yacc,2) + pow(zacc,2));

  Serial.print(xacc,3);
  Serial.print('\t');
  Serial.print(yacc,3);
  Serial.print('\t');
  Serial.print(zacc,3);
  Serial.print('\t');
  Serial.println(resultant,3);
  delay(100);
}


void setup() {
  /*
  Wire.begin();            //Open I2C communications as master
  Serial.begin(9600);    //Open serial communications to the PC to see what's happening
  */
  
  Serial.println(" ");
  delay(1500);
  Serial.println("Program begin...");
  delay(1000);
  Serial.println("This program will start the ESC.");

  motor1.attach(MOTOR_PIN1); 
  motor2.attach(MOTOR_PIN2);
  motor3.attach(MOTOR_PIN3);
  motor4.attach(MOTOR_PIN4);

  Wire.begin();
  Serial.begin(115200);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope

  /*
  Serial.print("Now writing maximum output: (");Serial.print(MAX_SIGNAL);Serial.print(" us in this case)");Serial.print("\n");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);
  motor3.writeMicroseconds(MAX_SIGNAL);
  motor4.writeMicroseconds(MAX_SIGNAL); 
  
  // Wait for input
  while (!Serial.available());
  Serial.read();

  // Send min output
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Sending minimum output: (");Serial.print(MIN_SIGNAL);Serial.print(" us in this case)");Serial.print("\n");
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL); 
  Serial.println("The ESC is calibrated");
  Serial.println("----");
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");


  writeTo(0x53,0x31,0x09); //Set accelerometer to 11bit, +/-4g
  writeTo(0x53,0x2D,0x08); //Set accelerometer to measure mode
  writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
  writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
  */
}


void loop() {
  acc();
  gyro(); 
 Serial.begin(115200);  /******************************************/
  Serial.println("Starting set up");

  int totalGyroXValues  = 0;
  int totalGyroYValues  = 0;
  int totalGyroZValues  = 0;
  int i;

  kalman_init(&pitch_data);
  kalman_init(&roll_data);
  Serial.println("kalman set up");

  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT);
  pinMode(MOTOR_PIN4, OUTPUT);

  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);
  motor3.attach(MOTOR_PIN3);
  motor4.attach(MOTOR_PIN4);

  Serial.println("motors set up");


//  // disable SD card slot on WiFi shield
//  pinMode(4, OUTPUT);
//  digitalWrite(4, HIGH);

  motorctl(1, MIN_SIGNAL);
  motorctl(2, MIN_SIGNAL);
  motorctl(3, MIN_SIGNAL);
  motorctl(4, MIN_SIGNAL);

  Serial.println("motor control set up");


  cThrottle = 0;
  cYaw     = 0;
  cRoll    = 0;
  cPitch   = 0;
  cCtrl    = 0;
  wbuf[0] = 'Q'; 
  wbuf[1] = 'R'; 
  wbuf[2] = 'O'; 
  wbuf[3] = 'N'; 
  wbuf[4] = 'E'; 


  Serial.println("MPU set up");
}

/*
void loop() {
  getGyroscopeReadings(gyroResult);
  getAccelerometerReadings(accelResult);

  Serial.print(gyroResult[0]);
  Serial.print("\t");
  Serial.print(gyroResult[1]);
  Serial.print("\t"); 
  Serial.print(gyroResult[2]);
  Serial.print("\t\t");
  Serial.print(accelResult[0]);
  Serial.print("\t");
  Serial.print(accelResult[1]);
  Serial.print("\t");
  Serial.print(accelResult[2]);
  Serial.print("\n");

  delay(50);
}

*/
