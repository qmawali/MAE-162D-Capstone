#include "Motor.h"

#define R_S_f 4    //ir sensor Right front
#define L_S_f 2    //ir sensor Left front
#define R_S_b 3    //ir sensor Right back
#define L_S_b 5    //ir sensor Left back

Motor motorA(22, 8);
Motor motorB(23, 10);
int pmw = 255;
bool dir = 0;

void setup() {

  pinMode(R_S_f, INPUT);
  pinMode(L_S_f, INPUT);
  pinMode(R_S_b, INPUT);
  pinMode(L_S_b, INPUT);
  Serial.begin(9600);
  delay(1000);

}

void loop() {

if ((dir = 0)){
if ((digitalRead(R_S_f) == 0) && (digitalRead(L_S_f) == 0)) {
    forward();
  }
  if ((digitalRead(R_S_f) == 1) && (digitalRead(L_S_f) == 0)) {
    turnRight();
  }
  if ((digitalRead(R_S_f) == 0) && (digitalRead(L_S_f) == 1)) {
    turnLeft();
  }
  if ((digitalRead(R_S_f) == 1) && (digitalRead(L_S_f) == 1)) {
    dir = 1;
    pmw = -1*pmw;
    Stop();
  }

}
else 

if ((digitalRead(R_S_b) == 0) && (digitalRead(L_S_b) == 0)) {
    forward();
  }
  if ((digitalRead(R_S_b) == 1) && (digitalRead(L_S_b) == 0)) {
    turnRight();
  }
  if ((digitalRead(R_S_b) == 0) && (digitalRead(L_S_b) == 1)) {
    turnLeft();
  }
  if ((digitalRead(R_S_b) == 1) && (digitalRead(L_S_b) == 1)) {
    Stop();
  }

}
void forward() {
  motorA.drive(pmw);
  motorB.drive(pmw);
}

void turnRight() {
  motorA.drive(-pmw);
  motorB.drive(pmw);
}

void turnLeft() {
  motorA.drive(pmw);
  motorB.drive(-pmw);
}

void Stop() {
  motorA.drive(0);
  motorB.drive(0);
}


