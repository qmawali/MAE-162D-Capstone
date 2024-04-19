#include "Motor.h"

#define R_S 14    //ir sensor Right
#define L_S 15    //ir sensor Left

#define pwma 100

Motor motorA(12, 11);
Motor motorB(10, 9);

void setup() {

  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);
  Serial.begin(9600);
  delay(1000);

}

void loop() {
  bool rIR = digitalRead(R_S);
  bool lIR = digitalRead(L_S);
  Serial.println(String(lIR) + ' ' + String(rIR));

  if ((rIR == 0) && (lIR == 0)) {
    forward();
  }
  if ((rIR == 1) && (lIR == 0)) {
    turnRight();
  }
  if ((rIR == 0) && (lIR == 1)) {
    turnLeft();
  }
  if ((rIR == 1) && (lIR == 1)) {
    Stop();
  }
  //delay(10);
}

void forward() {
  motorA.drive(pwma);
  motorB.drive(pwma);
}

void turnRight() {
  motorA.drive(-pwma);
  motorB.drive(pwma);
}

void turnLeft() {
  motorA.drive(pwma);
  motorB.drive(-pwma);
}

void Stop() {
  motorA.drive(0);
  motorB.drive(0);
}