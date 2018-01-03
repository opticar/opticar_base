#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

class Kinematic
{
public:
	struct OutputSpeeds
	{
		int percFrontLeft;
		int percFrontRight;
		int percRearLeft;
		int percRearRight;
		
		float dbg1;
		float dbg2;
		float dbg3;
	};
	
	struct InputSteering
	{
		float velX;
		float velY;
		float rotZ;
	};
	
	Kinematic(int maxRPM, float wheelDiameter, float baseWidth, float baseLength);
	
	OutputSpeeds calculateSpeeds(const InputSteering& steering);
	
protected:
  float m_MaxRPM;
	float m_WheelDiameter;
  float m_WheelRadius;
  float m_BaseWidth;
  float m_BaseLength;
};

#ifdef __cplusplus
}
#endif