#include <Wire.h> //The I2C library
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiNINA.h>
#include <SPI.h>
#include <Servo.h>

#define MPU6050 0x68 //Device address
#define ACCEL_CONFIG 0x1C //Accelerometer configuration address
#define GYRO_CONFIG 0x1B // GYRO_CONFIG register (1B hex)

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
#define MOTOR_PIN1 9
#define MOTOR_PIN2 10
#define MOTOR_PIN3 11
#define MOTOR_PIN4 12
#define THR_ANDROID_MIN 0
#define THR_ANDROID_MAX 255
#define MAX_TILT_THRUST_CORRECTION 1.3 // 30%
#define MAX_TILT_ANGLE 15
#define PI 3.14159
#define BUFSIZE 25 
#define PORT 5880         // UDP port

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


long  BatLvl = 0;
long  WifiAtt = 0;

// Commanded values (from Android)
byte     cThrottle;     
short         cYaw;
short        cRoll;
short       cPitch;
boolean    cCutoff; 



int   gyroResult[3], accelResult[3];  // Gyro/Accel; static accelerometer's biases measured on fixed drone with engines off
float biasGyroX = 0, biasGyroY = 0, biasGyroZ = 0;
float pitchGyro;
float pitchAccel;
float rollGyro;
float rollAccel;
float pitchGyroDelta;
float rollGyroDelta;
float pitchGyroRate;
float rollGyroRate;
float pitchError = 1.18; //0.34;   // Static accelerometer's readings on the flat horizontal surface
float rollError = 1.5; //3.99;     // i.e. errors due to not ideal IMU 6DOF chip position on the drone
float Rad = 0.01745;

float     Kp = 0;    // PID
float     Ki = 0;    // 2; 0.8; 0 - test OK
float     Kd = 0;
float   sumP = 0;
double  sumR = 0;
double prevR = 0;
double prevP = 0;
double  pidP = 0;
double  pidR = 0;


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

#define Q1 0.1       // Kalman filter 0.1/0.05/0.005
#define Q2 0.05
#define Q3 0.005
#define R1 6000      // 5000
#define R2 715       // 715
struct kalman_data
{
  float x1, x2, x3;
  float p11, p12, p13, p21, p22, p23, p31, p32, p33;
  float q1, q2, q3;
  float r1, r2;
};
kalman_data pitch_data;
kalman_data roll_data;
float pitchPrediction = 0;
float rollPrediction  = 0;

int      ThrBase = 0;          // Thrust
int      Thrust1 = MIN_SIGNAL;
int      Thrust2 = MIN_SIGNAL;
int      Thrust3 = MIN_SIGNAL;
int      Thrust4 = MIN_SIGNAL;
float    Yaw_factor = 1;

boolean            emerdst = 0;    // Emergency descent flag
unsigned long     sendtime = 0;    // Time since last UDP packet sent (ms)
unsigned short      sendfq = 200;  // Frequency of updates to Android (ms)
unsigned long   packettime = 0;    // Time since last UDP packet received (ms)
unsigned long   nocommtime = 1500; // Maximum incoming packets absence time (ms)
unsigned long prevlooptime = 0;
unsigned long     timeStep;        // Times since last loop ms, s
float            timeStepS;

/*
https://arduino.stackexchange.com/questions/38908/how-can-arduino-know-wire-available-is-true-or-false
https://forum.arduino.cc/t/imu-doesnt-work-for-vertical-position/442502
 
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

/**********************************/
/*           S E T U P            */
/**********************************/

void setup() {
  /*
  Wire.begin();            //Open I2C communications as master
  Serial.begin(9600);    //Open serial communications to the PC to see what's happening
  */
  
//  Serial.println(" ");
//  delay(1500);
//  Serial.println("Program begin...");
//  delay(1000);
//  Serial.println("This program will start the ESC.");

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

  Serial.println("motors is set up");

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
  wbuf[0] = 'D'; 
  wbuf[1] = 'R'; 
  wbuf[2] = 'O'; 
  wbuf[3] = 'N'; 
  wbuf[4] = 'E'; 


  Serial.println("MPU set up");

  Wire.begin();

  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope

  
  Serial.println("MPU set up");
  // Determine zero bias for all axes of both sensors by averaging 50 measurements
  delay(100); // let the drone stabilize after power switch click and sensors initialize
  prevlooptime = millis();

  for (i = 0; i < 50; i++)
  {
    getGyroscopeReadings(gyroResult);
    getAccelerometerReadings(accelResult);
    totalGyroXValues += gyroResult[0];
    totalGyroYValues += gyroResult[1];
    totalGyroZValues += gyroResult[2];
    delay(25);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;

  Serial.println("gyro set up");

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

  while (!Udp.begin(PORT))  {
    Serial.println("waiting for udp connection");
  }

  Serial.print("set up complete");

}



/**********************************/
/*       M A I N   L O O P        */
/**********************************/


void loop() {
  acc();
  gyro(); 

}


void kalman_init(struct kalman_data *data)   // Setup the kalman data struct
{
  data->x1 = 0.0f;
  data->x2 = 0.0f;
  data->x3 = 0.0f;

  // Init P to diagonal matrix with large values since
  // the initial state is not known
  data->p11 = 100.0f;
  data->p12 = 0;
  data->p13 = 0;
  data->p21 = 0;
  data->p22 = 100.0f;
  data->p23 = 0;
  data->p31 = 0;
  data->p32 = 0;
  data->p33 = 100.0f;

  data->q1 = Q1;
  data->q2 = Q2;
  data->q3 = Q3;
  data->r1 = R1;
  data->r2 = R2;
}
