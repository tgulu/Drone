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
void gyro(int gyroResult[]) {
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


void motorctl(unsigned short motor, unsigned short thr)
{
  thr = thr > MAX_SIGNAL ? MAX_SIGNAL : thr;
  thr = thr < MIN_SIGNAL ? MIN_SIGNAL : thr;
  if (motor == 1)
    motor1.writeMicroseconds(thr);
  else if (motor == 2)
    motor2.writeMicroseconds(thr);
  else if (motor == 3)
    motor3.writeMicroseconds(thr);
  else if (motor == 4)
    motor4.writeMicroseconds(thr);
}
//Function for reading the accelerometer.

void acc(int accelResult[]) {
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

  
//  Serial.println(" ");
//  delay(1500);
//  Serial.println("Program begin...");
//  delay(1000);
//  Serial.println("This program will start the ESC.");

  Serial.begin(115200);  
  Serial.println("Starting set up"); //Open serial communications to the PC to see what's happening

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
    getGyroscopeReadings(gyroResult);      // Read gyro and accelerometer sensors
    timeStep=millis()-prevlooptime;        // measure time
    timeStepS=timeStep/1000.0;
    prevlooptime=millis();
    getAccelerometerReadings(accelResult);
    pitchAccel =atan2(accelResult[1] / 256.0,accelResult[2] / 256.0) * 360.0 / (2*PI) - pitchError;
    rollAccel  =atan2(accelResult[0] / 256.0,accelResult[2] / 256.0) * 360.0 / (2*PI) - rollError;
  
     pitchGyroRate=(gyroResult[0] - biasGyroX) / 14.375;
      rollGyroRate=(gyroResult[1] - biasGyroY) / 14.375;
    pitchGyroDelta=pitchGyroRate*timeStepS;
     rollGyroDelta=rollGyroRate *timeStepS;
  
    // Kalman filter
    kalman_innovate(&pitch_data,pitchAccel, pitchGyroRate,timeStepS);
    kalman_innovate(&roll_data,  rollAccel,-rollGyroRate, timeStepS);
    pitchPrediction=pitch_data.x1;
    rollPrediction = roll_data.x1;
  
    //Waiting for incoming UDP packed
    // if there's data available, read a packet
    rbuf[0]=0;
    if (Udp.parsePacket()){
      Serial.println("GOT PACKET");
      badpacket=0;
      commstart=1;
      int len=Udp.read(rbuf, BUFSIZE);
      if (len>0)
      {
        rbuf[len] = 0;
        if (rbuf[0]=='D' && rbuf[1]=='R' && rbuf[2]=='O' && rbuf[3]=='N' && rbuf[4]=='E')
        {
          packettime=millis();
          emerdst=0;
          cThrottle=rbuf[5];
          cYaw=rbuf[8] & 0x00FF;
          cYaw-=127;
          Kp=*(float *)&rbuf[10];
          Kd=*(float *)&rbuf[14];
          Ki=*(float *)&rbuf[18];
          if ( ((short)(rbuf[6] & 0x00FF)>=0) && ((short)(rbuf[6] & 0x00FF)<=180) )
            cRoll=(rbuf[6]& 0x00FF)-90;
          else
            badpacket=1;
          if ( ((short)(rbuf[7] & 0x00FF)>=0) && ((short)(rbuf[7] & 0x00FF)<=180) )
            cPitch=(rbuf[7] & 0x00FF)-90;
          else
            badpacket=1;
          if ((rbuf[9]==0) || (rbuf[9]==1))
            cCtrl=rbuf[9];
          else
            badpacket=1;
          if ((rbuf[22]==0) || (rbuf[22]==1))
            cCutoff=rbuf[22];
          else
            badpacket=1;
        }
        else
          badpacket=1;
      }
    }
  
    if (!cCtrl)
    {
      cRoll =0;
      cPitch=0;
    }
    else
    {
      cPitch=(abs(cPitch)>MAX_TILT_ANGLE)?((abs(cPitch)/cPitch)*MAX_TILT_ANGLE):cPitch;
      cRoll = (abs(cRoll)>MAX_TILT_ANGLE)?((abs(cRoll) /cRoll) *MAX_TILT_ANGLE):cRoll;
    }
  
    // Emergency descent mode in case of comm loss
    if ((millis()-packettime>=nocommtime)  || (WiFi.status() != WL_CONNECTED))  // 4 - no WiFi or lost WiFi, 3 - OK, (re)connected after both cases
    {
      emerdst=1;
      cThrottle=EMERDSTPWR;
    }
  
    // PID values
    sumP+=(pitchPrediction+cPitch)*timeStepS;
    sumR+=(rollPrediction -cRoll) *timeStepS;
    pidP =Kp*(pitchPrediction+cPitch)+ Ki*sumP + Kd*pitchGyroRate;  // Gyro readings * Kd as non-0 gyro is rate of error's change!
    pidR =Kp*(rollPrediction-cRoll)  + Ki*sumR -  Kd*rollGyroRate;
  
    if (cCutoff) {
      Thrust1=MIN_SIGNAL;
      Thrust2=MIN_SIGNAL;
      Thrust3=MIN_SIGNAL;
      Thrust4=MIN_SIGNAL;
    }
    else
    {
      ThrBase=map(cThrottle,THR_ANDROID_MIN,THR_ANDROID_MAX,MIN_SIGNAL,MAX_SIGNAL);
      float TiltThrCorrection=sqrt(1 + tan(abs(pitchPrediction)*Rad)*tan(abs(pitchPrediction)*Rad) + tan(abs(rollPrediction)*Rad)*tan(abs(rollPrediction)*Rad));
      TiltThrCorrection=TiltThrCorrection > MAX_TILT_THRUST_CORRECTION ? MAX_TILT_THRUST_CORRECTION : TiltThrCorrection;
      ThrBase=ThrBase * TiltThrCorrection;
  
      Thrust1 = ThrBase - (int)cYaw - pidP + pidR;
      Thrust2 = ThrBase + (int)cYaw - pidP - pidR;
      Thrust3 = ThrBase - (int)cYaw + pidP - pidR;
      Thrust4 = ThrBase + (int)cYaw + pidP + pidR;
    }
    motorctl(1,Thrust1);
    motorctl(2,Thrust2);
    motorctl(3,Thrust3);
    motorctl(4,Thrust4);
  
    if (millis()-sendtime>=sendfq && commstart)
    {
      //BatLvl=readVcc();                 // 4892mV=full; disabled - takes additional 5ms
      wbuf[0]='D';
      wbuf[5]=0;
      wbuf[6]=(short)rollPrediction+90;
      wbuf[7]=0;
      wbuf[8]=(short)pitchPrediction+90;
      wbuf[9]=0;  //HDG>>8;               // Heading and altitude not there yet
      wbuf[10]=0; //HDG;
      wbuf[11]=0; //ALT>>8;
      wbuf[12]=0; //ALT;
      wbuf[13]=cThrottle;
      wbuf[14]=0;
      wbuf[15]=BatLvl>>8;
      wbuf[16]=BatLvl;
      wbuf[17]=cYaw;
      wbuf[18]=cCtrl;
      wbuf[19]=*(byte *)&timeStep;
      //sending UDP packets to the phone
      if (Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()))
      {
        Udp.write(wbuf, BUFSIZE);
        if (!Udp.endPacket())
        {
          send_failed=1;
        }
        else
        {
          send_failed=0;
          sendtime=millis();
        }
      }
      else
      {
        send_failed=1;
      }
    }
} 


//https://lhelge.se/2012/04/pitch-and-roll-estimating-kalman-filter-for-stabilizing-quadrocopters/
void kalman_innovate(kalman_data *data, float z1, float z2, float DT)
{
  float y1, y2;
  float a, b, c;
  float sDet;
  float s11, s12, s21, s22;
  float k11, k12, k21, k22, k31, k32;
  float p11, p12, p13, p21, p22, p23, p31, p32, p33;

  // Step 1
  // x(k) = Fx(k-1) + Bu + w:
  data->x1 = data->x1 + DT * data->x2 - DT * data->x3;
  //x2 = x2;
  //x3 = x3;

  // Step 2
  // P = FPF'+Q
  a = data->p11 + data->p21 * DT - data->p31 * DT;
  b = data->p12 + data->p22 * DT - data->p32 * DT;
  c = data->p13 + data->p23 * DT - data->p33 * DT;
  data->p11 = a + b * DT - c * DT + data->q1;
  data->p12 = b;
  data->p13 = c;
  data->p21 = data->p21 + data->p22 * DT - data->p23 * DT;
  data->p22 = data->p22 + data->q2;
  //p23 = p23;
  data->p31 = data->p31 + data->p32 * DT - data->p33 * DT;
  //p32 = p32;
  data->p33 = data->p33 + data->q3;

  // Step 3
  // y = z(k) - Hx(k)
  y1 = z1 - data->x1;
  y2 = z2 - data->x2;

  // Step 4
  // S = HPT' + R
  s11 = data->p11 + data->r1;
  s12 = data->p12;
  s21 = data->p21;
  s22 = data->p22 + data->r2;

  // Step 5
  // K = PH*inv(S)
  sDet = 1 / (s11 * s22 - s12 * s21);
  k11 = (data->p11 * s22 - data->p12 * s21) * sDet;
  k12 = (data->p12 * s11 - data->p11 * s12) * sDet;
  k21 = (data->p21 * s22 - data->p22 * s21) * sDet;
  k22 = (data->p22 * s11 - data->p21 * s12) * sDet;
  k31 = (data->p31 * s22 - data->p32 * s21) * sDet;
  k32 = (data->p32 * s11 - data->p31 * s12) * sDet;

  // Step 6
  // x = x + Ky
  data->x1 = data->x1 + k11 * y1 + k12 * y2;
  data->x2 = data->x2 + k21 * y1 + k22 * y2;
  data->x3 = data->x3 + k31 * y1 + k32 * y2;

  // Step 7
  // P = (I-KH)P
  p11 = data->p11 * (1.0f - k11) - data->p21 * k12;
  p12 = data->p12 * (1.0f - k11) - data->p22 * k12;
  p13 = data->p13 * (1.0f - k11) - data->p23 * k12;
  p21 = data->p21 * (1.0f - k22) - data->p11 * k21;
  p22 = data->p22 * (1.0f - k22) - data->p12 * k21;
  p23 = data->p23 * (1.0f - k22) - data->p13 * k21;
  p31 = data->p31 - data->p21 * k32 - data->p11 * k31;
  p32 = data->p32 - data->p22 * k32 - data->p12 * k31;
  p33 = data->p33 - data->p23 * k32 - data->p13 * k31;
  data->p11 = p11;
  data->p12 = p12;
  data->p13 = p13;
  data->p21 = p21;
  data->p22 = p22;
  data->p23 = p23;
  data->p31 = p31;
  data->p32 = p32;
  data->p33 = p33;
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
