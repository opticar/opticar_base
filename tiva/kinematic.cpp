#include "kinematic.h"

#include <math.h>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265
#endif

static float clamp(float val, float min, float max)
{
  if (val > max) return max;
  if (val < min) return min;
  return val;
}

Kinematic::Kinematic(int maxRPM, float wheelDiameter, float baseWidth, float baseLength) :
  m_MaxRPM(maxRPM),
  m_WheelDiameter(wheelDiameter),
  m_BaseWidth(baseWidth),
  m_BaseLength(baseLength)
{
	m_WheelRadius = m_WheelDiameter / 2;
}

Kinematic::OutputSpeeds Kinematic::calculateSpeeds(const InputSteering& steering)
{	
  OutputSpeeds result;
  
  static const float radPerSecToRPM = 60.0f / (2 * M_PI);
  
  // Calculate wheel speeds
  float frontLeftRadPerSec  = (1 / m_WheelRadius) * (steering.velX - steering.velY - (m_BaseWidth + m_BaseLength) * steering.rotZ);
  float frontRightRadPerSec = (1 / m_WheelRadius) * (steering.velX + steering.velY + (m_BaseWidth + m_BaseLength) * steering.rotZ);
  float rearLeftRadPerSec   = (1 / m_WheelRadius) * (steering.velX + steering.velY - (m_BaseWidth + m_BaseLength) * steering.rotZ);
  float rearRightRadPerSec  = (1 / m_WheelRadius) * (steering.velX - steering.velY + (m_BaseWidth + m_BaseLength) * steering.rotZ);
    
  float frontLeftRPM  = frontLeftRadPerSec  * radPerSecToRPM;
  float frontRightRPM = frontRightRadPerSec * radPerSecToRPM;
  float rearLeftRPM   = rearLeftRadPerSec   * radPerSecToRPM;
  float rearRightRPM  = rearRightRadPerSec  * radPerSecToRPM;
  
  result.percFrontLeft  = clamp(100 * frontLeftRPM  / m_MaxRPM, -100, 100);
  result.percFrontRight = clamp(100 * frontRightRPM / m_MaxRPM, -100, 100);
  result.percRearLeft   = clamp(100 * rearLeftRPM   / m_MaxRPM, -100, 100);
  result.percRearRight  = clamp(100 * rearRightRPM  / m_MaxRPM, -100, 100);
	
	result.dbg1 = frontLeftRPM / m_MaxRPM;
  result.dbg2 = m_MaxRPM;
  result.dbg3 = frontLeftRPM;
  
  return result;
}