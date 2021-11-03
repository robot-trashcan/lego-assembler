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

int toMove;
int pos;

int positions[] = {
  1500, 1500, 1500, 1500, 1500, 1500
};

void reset_position() {
  myse.moveServos(6,2000,1,1500,2,1500,3,1500,4,1500,5,1500,6,1500);
  delay(2000);
}

void reset_position_noclaw() {
  myse.moveServos(5,2000,2,1500,3,1500,4,1500,5,1500,6,1500);
  delay(2000);
}

void crane_position() {
  myse.moveServos(3, 2000, 3, 2500, 4, 900, 5, 1300);
  delay(2000);
}
void test(int x) {
  myse.moveServos(2, 1000, 2, 1500+x, 6, 1500+x);
  delay(1000);
}

void close_claw() {
  myse.moveServos(1, 1000, 1, 2400);
  delay(1000);
}

void setup() {
  pinMode(13,OUTPUT);
  mySerial.begin(9600);  // opens software serial port, sets data rate to 9600 bps
  Serial.begin(9600);

  reset_position();
}

void loop() {
  while(!Serial.available());
  toMove = Serial.parseInt();
  pos = Serial.parseInt();
  myse.moveServo(toMove, pos, 1000);
  positions[toMove-1] = pos;
  for(int i = 0; i < 6; i++) {
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(positions[i]);
    Serial.print(", ");
  }
  Serial.print("\n");
}
