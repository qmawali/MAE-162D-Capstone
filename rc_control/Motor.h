#ifndef Motor_h
#define Motor_h
#include "Arduino.h"

class Motor {
public:
  Motor(int dirPin, int PWM); // Constructor

  void setPWM(int newPWM);
  void run(int time);
  void toggleDir();
  void setDir(bool newDir);

  // Expects +/- PWM inputs, for open-loop control
  void drive(int target);

private:
  // Pins
  int dirPin, PWM;

  // Trackers
  bool dir;
};

#endif