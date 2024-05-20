#ifndef Motor2PWM_h
#define Motor2PWM_h
#include "Arduino.h"

class Motor {
public:
  Motor(int PWM1, int PWM2, int maxPWM = 255); // Constructor

  void setPWM(int newPWM);
  void run(int time);
  void toggleDir();
  void setDir(bool newDir);

  // Expects +/- PWM inputs, for open-loop control
  void drive(int target);

private:
  // Pins
  int PWM1, PWM2, PWM;

  int maxPWM;

  void writeDir(bool dir);
};

#endif