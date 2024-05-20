#include "Joystick.h"
#include "Motor2PWM.h"

// maxPWM not 255 because DBH-12V motor driver requires <98% demand cycle
#define maxPWM 250
Joystick jsA(A0, maxPWM);
Joystick jsB(A1, maxPWM);

Motor motorA(11, 12, maxPWM);
Motor motorB(7, 8, maxPWM);

void setup() {
  Serial.begin(9600);
  //while(!Serial);

  Serial.println(jsA.calibrate());
  Serial.println(jsB.calibrate());
  delay(500);
  motorA.drive(0);
  motorB.drive(0);
  delay(500);

}

void loop() {
  int straight = jsA.read();
  int turn = jsB.read();

  motorA.drive(straight + turn);
  motorB.drive(straight - turn);

  Serial.println(String(straight) + ' ' + String(turn));
}
