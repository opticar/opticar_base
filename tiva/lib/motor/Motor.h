#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#include <opticar_config.h>

#include <DemoLED.h>
#include <ros.h>

class Controller
{
public:
  Controller(int configDataIndex, const DemoLED::LEDBlock ledBlock);
  void init(ros::NodeHandle& nh);
  void spin(float pwmPercentage);

private:
  int configDataIndex;
  bool initialized;
  static uint32_t pwmPeriod;
  DemoLED::LEDBlock ledBlock;
  int currentLogicalDirection;
};

#endif
