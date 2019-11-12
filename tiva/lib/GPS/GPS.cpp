#include "GPS.h"

#include <cmath>

GPS BaseGPS(1);

GPS::GPS(int serialNr)
{
  switch (serialNr)
  {
    case 0:
      serial = &Serial;
    case 1:
      serial = &Serial1;
      break;
    case 2:
      serial = &Serial2;
      break;
    case 3:
      serial = &Serial3;
      break;
    case 4:
      serial = &Serial4;
      break;
    case 5:
      serial = &Serial5;
      break;
    case 6:
      serial = &Serial6;
      break;
    case 7:
      serial = &Serial7;
      break;
    default:
      serial = nullptr;
  }

  serial->begin(9600);
}

void GPS::step()
{
  while (serial->available() > 0)
  {
    gps.encode(serial->read());
  }
}

void GPS::updateMessage(sensor_msgs::NavSatFix& msg)
{
  if (gps.location.isValid())
  {
    msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    msg.status.service = 1;

    msg.latitude = gps.location.lat();
    msg.longitude = gps.location.lng();

    if (gps.altitude.isValid())
    {
      msg.altitude = gps.altitude.value();
    }
    else
    {
      msg.altitude = std::nan("");
    }

    for (auto& covariance : msg.position_covariance)
    {
      covariance = 0;
    }

    msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }
  else
  {
    msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    msg.status.service = 0;

    msg.latitude = 0;
    msg.longitude = 0;
    msg.altitude = std::nan("");

    for (auto& covariance : msg.position_covariance)
    {
      covariance = 0;
    }

    msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }
}