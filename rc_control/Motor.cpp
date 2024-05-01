#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int dirPin, int PWM) {
  // Pins
  this->dirPin = dirPin;
  this->PWM = PWM;

  pinMode(dir, OUTPUT);
  pinMode(PWM, OUTPUT);

  // Initial Direction, Power
  digitalWrite(dirPin, LOW);
  this->dir = false;
  //analogWrite(PWM, 0); // This line ensures cosntant 0 PWM output on nano 33 ble, not sure why
}

void Motor::setPWM(int newPWM){
  analogWrite(PWM, newPWM);
}

void Motor::run(int time){
  delay(time);
  analogWrite(PWM, 0);
}

void Motor::toggleDir(){
  digitalWrite(dirPin, !dir);
  dir = !dir;
}

void Motor::setDir(bool newDir) {
  if(dir!=newDir) {
    dir = newDir;
    digitalWrite(dirPin, dir);
  }
}

void Motor::drive(int target) {
  this->setDir(target>0);
  this->setPWM(min(abs(target), 255));
}