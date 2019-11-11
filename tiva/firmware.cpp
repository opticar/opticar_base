// This firmware was adapted from the linorobot teensy firmware
// Available at
// https://github.com/linorobot/linorobot/blob/master/teensy/firmware/src/firmware.ino

#include <Energia.h>

#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "tivac_hardware.h"

// Various configuration settings
#include <opticar_config.h>

// Various drivers
#include <DemoLED.h>
#include <Imu.h>
#include <SystemLED.h>

#include <ros.h>
#include <ros/time.h>
// header file for publishing velocities for odom
#include <opticar_msgs/Velocities.h>
// header file for cmd_subscribing to "cmd_vel"
#include <geometry_msgs/Twist.h>
// header file for pid server
#include <opticar_msgs/PID.h>
// header file for imu
#include <opticar_msgs/IMU.h>

unsigned long g_PrevCommandTime = 0;  // ms

// Callbacks for the message subscribers
void commandCallback(const geometry_msgs::Twist &twist);
void pidCallback(const opticar_msgs::PID &pid);

// Other forward declarations
void printDebug();
void publishIMU();

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmdSub("cmd_vel", commandCallback);
ros::Subscriber<opticar_msgs::PID> pidSub("pid", pidCallback);

opticar_msgs::IMU rawImuMsg;
ros::Publisher rawImuPub("raw_imu", &rawImuMsg);

opticar_msgs::Velocities rawVelMsg;
ros::Publisher rawVelPub("raw_vel", &rawVelMsg);

extern "C"
{
  void SysTickDispatcher();
  // From Energia
  void SysTickIntHandler(void);
}

void SysTickDispatcher()
{
  // Call both downstream handlers
  SysTickIntHandler();
  TivaCHardware::SystickIntHandler();
}

void setup()
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();

  // Initialize on-board LEDs
  SystemLED::init();
  // Switch on heartbeat LED to indicate we are waiting for a connection
  LedHeartbeat.on();

  // Initialize demonstrator LEDs
  DemoLed.init();

  // Initialize communication
  nh.initNode();

  // Reroute system tick handler to our dispatcher
  SysTickIntDisable();
  SysTickIntUnregister();
  SysTickIntRegister(SysTickDispatcher);
  SysTickIntEnable();

  // Subscribe to the necessary topics
  nh.subscribe(cmdSub);
  nh.subscribe(pidSub);

  // Advertise our sensor data
  nh.advertise(rawImuPub);
  nh.advertise(rawVelPub);

  while (!nh.connected())
  {
    nh.spinOnce();
  }

  nh.loginfo("Opticar ECU connected");
  LedReady.on();

  DemoLed.setBlockColor(DemoLED::LED_BLOCK_VL, DemoLED::LED_BLUE);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_VR, DemoLED::LED_BLUE);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_HL, DemoLED::LED_BLUE);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_HR, DemoLED::LED_BLUE);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_INDL, DemoLED::LED_BLUE);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_INDR, DemoLED::LED_BLUE);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_REAR, DemoLED::LED_BLUE);

  delay(1);
}

void loop()
{
  static unsigned long prevControlTime = 0;    // ms
  static unsigned long prevImuTime = 0;        // ms
  static unsigned long prevGpsTime = 0;        // ms
  static unsigned long prevDebugTime = 0;      // ms
  static unsigned long prevHeartbeatTime = 0;  // ms
  static bool imuIsInitialized = false;

  if (millis() - prevImuTime >= (1000 / IMU_PUBLISH_RATE))
  {
    if (!imuIsInitialized)
    {
      imuIsInitialized = initIMU();

      if (imuIsInitialized)
      {
        nh.loginfo("IMU initialized");
      }
      else
      {
        nh.logfatal("IMU failed to initialize. Check IMU connection.");
      }
    }
    else
    {
      publishIMU();
    }

    prevImuTime = millis();
  }

  if (DEBUG)
  {
    if (millis() - prevDebugTime >= (1000 / DEBUG_RATE))
    {
      printDebug();
      prevDebugTime = millis();
    }
  }

  if (millis() - prevHeartbeatTime >= (1000 / HEARTBEAT_RATE))
  {
    LedHeartbeat.toggle();
    prevHeartbeatTime = millis();
  }

  nh.spinOnce();
}

void commandCallback(const geometry_msgs::Twist &twist)
{
}

void pidCallback(const opticar_msgs::PID &pid)
{
}

void publishIMU()
{
  rawImuMsg.acceleration = readAccelerometer();
  rawImuMsg.rotation = readGyroscope();
  rawImuMsg.magnetic = readMagnetometer();

  rawImuPub.publish(&rawImuMsg);
}

void printDebug()
{
  char buffer[50];
  sprintf(buffer, "Debug chatter");
  nh.loginfo(buffer);
}