#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#include OPTICAR_CFG

#include <DemoLED.h>
#include <ros.h>

class Controller
{
public:
  Controller(int configDataIndex, const DemoLED::LEDBlock ledBlock);
  void init(ros::NodeHandle& nh);
  void spin(float pwmPercentage);
  int getCurrentDirection();

  static const uint32_t DIR_FORWARD = 0;
  static const uint32_t DIR_BACKWARD = 1;

private:
  int configDataIndex;
  bool initialized;
  static uint32_t pwmPeriod;
  DemoLED::LEDBlock ledBlock;
  int currentLogicalDirection;
};

#endif
