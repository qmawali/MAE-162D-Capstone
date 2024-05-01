#include "Joystick.h"
#include "Motor.h"

Joystick jsA(A0, 255);
Joystick jsB(A1, 255);

Motor motorA(22, 8);
Motor motorB(23, 10);

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial.println(jsA.calibrate());
  Serial.println(jsB.calibrate());
  delay(500);

}

void loop() {
  float straight = jsA.read();
  float turn = jsB.read();

  motorA.drive(straight + turn);
  motorB.drive(straight - turn);

  Serial.println(String(straight) + ' ' + String(turn));
}
