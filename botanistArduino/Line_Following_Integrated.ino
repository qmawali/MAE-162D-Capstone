#include "Motor.h"

#define R_S 4    //ir sensor Right
#define L_S 2    //ir sensor Left

Motor motorA(22, 8);
Motor motorB(23, 10);

void setup() {

  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);
  Serial.begin(9600);
  delay(1000);

}

void loop() {

if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 0)) {
    forward();
  }
  if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 0)) {
    turnRight();
  }
  if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 1)) {
    turnLeft();
  }
  if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 1)) {
    Stop();
  }
  delay(10);
}

void forward() {
  motorA.drive(255);
  motorB.drive(255);
}

void turnRight() {
  motorA.drive(-255);
  motorB.drive(255);
}

void turnLeft() {
  motorA.drive(255);
  motorB.drive(-255);
}

void Stop() {
  motorA.drive(0);
  motorB.drive(0);
}