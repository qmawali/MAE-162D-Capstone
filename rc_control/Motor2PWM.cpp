#include "Arduino.h"
#include "Motor2PWM.h"

Motor::Motor(int PWM1, int PWM2) {
  // Pins
  this->PWM1 = PWM1;
  this->PWM2 = PWM2;

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Initial Direction, Power
  this->PWM = PWM1;
  //analogWrite(PWM, 0); // This line ensures cosntant 0 PWM output on nano 33 ble, not sure why
}

void Motor::setPWM(int newPWM){
  analogWrite(PWM, min(abs(newPWM), 255));
}

void Motor::run(int time){
  delay(time);
  analogWrite(PWM, 0);
}

void Motor::toggleDir(){
  if(PWM==PWM1)
    PWM = PWM2;
  else
    PWM = PWM1;
}

void Motor::setDir(bool newDir) {
  if((PWM==PWM1) != newDir)
      if(newDir)
        PWM = PWM1;
      else
        PWM = PWM2;
}

void Motor::drive(int target) {
  this->setDir(target>0);
  this->setPWM(min(abs(target), 255));
}