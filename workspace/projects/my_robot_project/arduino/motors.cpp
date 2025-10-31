#include "motors.h"

void Motors::init() {
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
}

void Motors::setSpeed(int left, int right) {
  analogWrite(leftPin, left);
  analogWrite(rightPin, right);
}
