#include "Arduino.h"
#include "Motor2PWM.h"

Motor::Motor(int PWM1, int PWM2, int maxPWM) {
  // Pins
  this->PWM1 = PWM1;
  this->PWM2 = PWM2;

  this->maxPWM = maxPWM;

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Initial Direction, Power
  this->PWM = PWM1;
  //analogWrite(PWM, 0); // This line ensures cosntant 0 PWM output on nano 33 ble, not sure why
}

void Motor::setPWM(int newPWM){
  analogWrite(PWM, min(abs(newPWM), maxPWM));
}

void Motor::run(int time){
  delay(time);
  analogWrite(PWM, 0);
}

void Motor::toggleDir(){
  if(PWM==PWM1)
    writeDir(0);
  else
    writeDir(1);
}

void Motor::setDir(bool newDir) {
  if((PWM==PWM1) != newDir)
      if(newDir)
        writeDir(1);
      else
        writeDir(0);
}

void Motor::drive(int target) {
  this->setDir(target>0);
  this->setPWM(min(abs(target), maxPWM));
}

void Motor::writeDir(bool dir) {
  if(dir) {
    PWM = PWM1;
    analogWrite(PWM2, 0);
  }
  else {
    PWM = PWM2;
    analogWrite(PWM1, 0);
  }
}