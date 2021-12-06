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

int servo_positions[] = {1500, 1500, 1500, 1500, 1500, 1500};

void reset_position() {
  myse.moveServos(6,2000,1,1500,2,1500,3,1500,4,1500,5,1500,6,1500);
  delay(2000);
}

void setup() {
  pinMode(13,OUTPUT);
  mySerial.begin(9600);  // opens software serial port, sets data rate to 9600 bps
  Serial.begin(115200);

  reset_position();
}

void loop() {
  while(!Serial.available());
  // read in values from pi
  for(int i = 0; i < 6; i++) {
    servo_positions[i] = Serial.readStringUntil('\0').toInt();
    Serial.print(servo_positions[i]);
    Serial.print(" ");
  }

  // move servos accordingly
  myse.moveServos(6, 1500, 1, servo_positions[0], 2, servo_positions[1], 3, servo_positions[2],
                           4, servo_positions[3], 5, servo_positions[4], 6, servo_positions[5]);
}
