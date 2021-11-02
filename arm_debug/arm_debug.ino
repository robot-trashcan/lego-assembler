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

void setup() {
  pinMode(13,OUTPUT);
  mySerial.begin(9600);  // opens software serial port, sets data rate to 9600 bps
  Serial.begin(9600);

  reset_position();
}

void loop() {
  char c;
  while(!Serial.available());
  toMove = 1;
  Serial.readBytes(&c, 1);
  switch(c) {
    case '1':
      toMove = 1;
      break;
    case '2':
      toMove = 2;
      break;
    case '3':
      toMove = 3;
      break;
    case '4':
      toMove = 4;
      break;
    case '5':
      toMove = 5;
      break;
    case '6':
      toMove = 6;
      break;
    case '+':
      positions[toMove-1] += 10;
      myse.moveServo(toMove, positions[toMove-1], 1000);
      delay(1000);
      break;
    case '-':
      positions[toMove-1] -= 10;
      myse.moveServo(toMove, positions[toMove-1], 1000);
      delay(1000);
      break;
    case 's':
      for(int i = 0; i < 6; i++) {
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(positions[i]);
        Serial.print(", ");
      }
      Serial.print("\n");
      break;
  }
}
