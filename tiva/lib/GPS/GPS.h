#ifndef OPTICAR_GPS_H
#define OPTICAR_GPS_H

#include "Arduino.h"

#include <ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <TinyGPS++.h>

class GPS
{
public:
  GPS(int serialNr);

  void step();
  void updateMessage(sensor_msgs::NavSatFix& msg);

private:
  HardwareSerial* serial;
  TinyGPSPlus gps;
};

extern GPS BaseGPS;

#endif