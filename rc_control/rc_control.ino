#include "Joystick.h"
#include "Motor2PWM.h"

Joystick jsA(A0, 255);
Joystick jsB(A1, 255);

Motor motorA(4, 5);
Motor motorB(6, 7);

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial.println(jsA.calibrate());
  Serial.println(jsB.calibrate());
  delay(500);
  motorA.drive(0);
  motorB.drive(0);
  delay(2000);

}

void loop() {
  int straight = jsA.read();
  int turn = jsB.read();

  motorA.drive(straight);
  motorB.drive(straight);

  Serial.println(String(straight) + ' ' + String(turn));
}
