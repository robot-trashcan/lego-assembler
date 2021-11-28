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

void reset_position() {
  myse.moveServos(6,2000,1,1500,2,1500,3,1500,4,1500,5,1500,6,1500);
  delay(2000);
}

void reset_position_noclaw() {
  myse.moveServos(5,2000,2,1500,3,1500,4,1500,5,1500,6,1500);
  delay(2000);
}

void crane_position() {
  myse.moveServos(3, 2000, 3, 2450, 4, 1080, 5, 1100);
  delay(2000);
}

void test() {
  myse.moveServos(2, 1500, 5, 1500, 6, 1500);
  delay(1500);
  crane_position();
  myse.moveServos(2, 1000, 2, 1800, 6, 1720);
  delay(1000);
  myse.moveServo(4, 850, 1000);
  delay(1000);
  myse.moveServo(1, 1500, 1000);
}
void twist(int x) {
  myse.moveServos(2, 1000, 2, 1500+x, 6, 1500+x);
  delay(1000);
}

void close_claw() {
  myse.moveServos(1, 1000, 1, 2500);
  delay(1000);
}

void wait() {
  while(!Serial.available());
  Serial.read();
}

void setup() {
  pinMode(13,OUTPUT);
  mySerial.begin(9600);  // opens software serial port, sets data rate to 9600 bps
  Serial.begin(9600);
  digitalWrite(13,HIGH); 

  // test(300);

  // delay(500);

  // close_claw();

  // reset_position_noclaw();

  reset_position();
  delay(3000);
  crane_position();
  twist(-200);
  close_claw();
  test(); 
}

void loop() {
}
