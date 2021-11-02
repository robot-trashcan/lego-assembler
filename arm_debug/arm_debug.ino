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

  // reset_position();
  toMove = 1;
    myse.moveServos(4, 2000, 3, 2430, 4, 1080, 5, 1025, 6, 1400);
    delay(2000);
    positions[3-1] = 2430;
    positions[4-1] = 1080;
    positions[5-1] = 1025;
    positions[6-1] = 1400;
}

void move_to(int servo, int pos_difference) {
  positions[servo-1] += pos_difference;
  myse.moveServo(servo, positions[servo-1], abs(pos_difference)*10);
  delay(pos_difference*10);
}

void loop() {
  char c;
  while(!Serial.available());
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
      move_to(toMove, 10);
      break;
    case '-':
      move_to(toMove, -10);
      break;
    case ']':
      move_to(toMove, 100);
      break;
    case '[':
      move_to(toMove, -100);
      break;
    case '\n':
      for(int i = 0; i < 6; i++) {
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(positions[i]);
        Serial.print(", ");
     }
     Serial.print("\n");
     break;
    default:
      break;
  }

}
