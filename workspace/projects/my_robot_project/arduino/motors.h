#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

class Motors {
private:
  int leftPin;
  int rightPin;
  
public:
  Motors(int left, int right) : leftPin(left), rightPin(right) {}
  void init();
  void setSpeed(int left, int right);
};

#endif
