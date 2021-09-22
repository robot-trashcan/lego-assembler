#include <LobotServoController.h>

/*
 * servo 1: claw
 * servo 2: claw twister
 * servo 3: top joint
 * servo 4: middle joint
 * servo 5: bottom joint
 * servo 6: rotation bus
 */

#define rxPin 10
#define txPin 11

SoftwareSerial mySerial(rxPin, txPin);
LobotServoController myse(mySerial);

void setup() {
  pinMode(13,OUTPUT);
  mySerial.begin(9600);  //opens software serial port, sets data rate to 9600 bps
  Serial.begin(9600);
  digitalWrite(13,HIGH);
  // myse.moveServos(6,1000,0,1000,1,1000,2,1000,3,1000,4,1000,5,1000);
  myse.moveServos(6,1000,0,1500,1,1500,2,1500,3,1500,4,1500,5,1500); 

  /*
  myse.moveServos(5,1000,0,1300,2,700,4,600,6,900,8,790); 
  //Control 5 servos, action time is 1000ms, move No.0 servo to 1300 position, move No.2 servo to 700 position, move No.4 servo to 600 position
  //Move No.6 servo to 900 position, move No.8 servo to 790 position
  delay(2000);

  LobotServo servos[2];   //an array of struct LobotServo
  servos[0].ID = 2;       //No.2 servo
  servos[0].Position = 1400;  //1400 position
  servos[1].ID = 4;          //No.4 servo
  servos[1].Position = 700;  //700 position
  myse.moveServos(servos,2,1000);  //control 2 servos, action time is 1000ms, ID and position are specified by the structure array "servos"
  */
  
}

void loop() {
  // myse.moveServo(6, 1500, 1000);
  myse.moveServos(3, 1000, 3, 2500, 4, 800, 5, 1400);
  delay(3000);
  myse.moveServo(1, 2400, 500);
  delay(1000);
  // myse.moveServo(6, 1700, 1000);
  myse.moveServos(3, 1000, 3, 1500, 4, 1500, 5, 1500);
  delay(2000);
  myse.moveServo(1, 1500, 500);
  delay(3000);
}
