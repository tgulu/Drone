

/*ESC calibration*/
#include <Servo.h>

//max & min power sent to motors
#define SIGNAL_MAX 2000
#define SIGNAL_MIN 1000

//motor pin set up
#define MOTOR_PIN1 9   /*  1   2   */
#define MOTOR_PIN2 10  /*   \ /    */
#define MOTOR_PIN3 11  /*   / \    */
#define MOTOR_PIN4 12  /*  3   4   */




int serialInput = 1000;


Servo motor1;             /* Motors (servos), numbering:   1   2   */
Servo motor2;             /*                                \ /    */
Servo motor3;             /*                                / \    */
Servo motor4;             /*                               3   4   */


void setup() {
  //set serial monitor to baud 9600 and print out begin message
  Serial.begin(9600);
  Serial.println(" ");
  delay(1500);
  Serial.println("Program begin...");
  delay(1000);
  Serial.println("This program will start the ESC.");
 
  motor1.attach(MOTOR_PIN1); 
  motor2.attach(MOTOR_PIN2);
  motor3.attach(MOTOR_PIN3);
  motor4.attach(MOTOR_PIN4);

  Serial.print("Now writing maximum output: (");Serial.print(SIGNAL_MAX);Serial.print(" us in this case)");Serial.print("\n");
  Serial.println("Turn on power source, then wait 2 seconds and press any key."); //plug in the Lipo battery
  
  //set the motors to 2000 
  motor1.writeMicroseconds(SIGNAL_MAX);
  motor2.writeMicroseconds(SIGNAL_MAX);
  motor3.writeMicroseconds(SIGNAL_MAX);
  motor4.writeMicroseconds(SIGNAL_MAX); 
  
  // Wait for input
  while (!Serial.available());
  Serial.read();

  // Send min output
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Sending minimum output: (");Serial.print(SIGNAL_MIN);Serial.print(" us in this case)");Serial.print("\n");

  //set the motors to 1000 
  motor1.writeMicroseconds(SIGNAL_MIN);
  motor2.writeMicroseconds(SIGNAL_MIN);
  motor3.writeMicroseconds(SIGNAL_MIN);
  motor4.writeMicroseconds(SIGNAL_MIN); 
  Serial.println("The ESC is calibrated");
  Serial.println("----");
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");

}

void loop() {
   
  if (Serial.available() > 0)
  {
    int serialInput = Serial.parseInt();  //Looks for the next valid integer in the incoming serial.
    if (serialInput > 999)
    {
      
      motor1.writeMicroseconds(serialInput);
      motor2.writeMicroseconds(serialInput);
      motor3.writeMicroseconds(serialInput);
      motor4.writeMicroseconds(serialInput);
      float motorSpeed = (serialInput-1000)/10;
      Serial.print("\n");
      Serial.println("Current motor speed:"); Serial.print("  "); Serial.print(motorSpeed); Serial.print("%"); 
    }     
  }
}
