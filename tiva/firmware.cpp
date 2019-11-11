// This firmware was adapted from the linorobot teensy firmware
// Available at
// https://github.com/linorobot/linorobot/blob/master/teensy/firmware/src/firmware.ino

#include <Energia.h>

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"

// Various configuration settings
#include <opticar_config.h>

// Various drivers
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

unsigned long g_PrevCommandTime = 0; // ms

// Callbacks for the message subscribers
void commandCallback(const geometry_msgs::Twist &twist);
void PidCallback(const opticar_msgs::PID &pid);

// Other forward declarations
void printDebug();

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmdSub("cmd_vel", commandCallback);
ros::Subscriber<opticar_msgs::PID> pidSub("pid", PidCallback);

opticar_msgs::IMU rawImuMsg;
ros::Publisher rawImuPub("raw_imu", &rawImuMsg);

opticar_msgs::Velocities rawVelMsg;
ros::Publisher rawVelPub("raw_vel", &rawVelMsg);

void logAddress(const char *name, void *addr)
{
  char buffer[50];
  sprintf(buffer, "Address of '%s': %p", name, addr);
  nh.loginfo(buffer);
}
void logFAddress(const char *name, void (*addr)())
{
  char buffer[50];
  sprintf(buffer, "Address of '%s': %p", name, addr);
  nh.loginfo(buffer);
}

void setup()
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();

  // Initialize on-board LEDs
  SystemLED::init();

  // Initialize communication
  nh.initNode();

  // Subscribe to the necessary topics
  nh.subscribe(cmdSub);
  nh.subscribe(pidSub);

  // Advertise our sensor data
  nh.advertise(rawImuPub);
  nh.advertise(rawVelPub);

  Led1.on();
  Led7.on();

  logAddress("null", NULL);
  extern uint32_t _estack;
  logAddress("stack", &_estack);

  extern void (*const g_pfnVectors[])(void);
  logFAddress("stacktable", g_pfnVectors[0]);
  logFAddress("reset", g_pfnVectors[1]);
  logFAddress("nmi", g_pfnVectors[2]);
  logFAddress("hardfault", g_pfnVectors[3]);
  logFAddress("default_mpu", g_pfnVectors[4]);
  logFAddress("systick12", g_pfnVectors[12]);
  logFAddress("systick13", g_pfnVectors[13]);
  logFAddress("systick14", g_pfnVectors[14]);
  logFAddress("systick15", g_pfnVectors[15]);
  logFAddress("systick16", g_pfnVectors[16]);
  logFAddress("systick17", g_pfnVectors[17]);

  while (!nh.connected())
  {
    nh.spinOnce();
  }

  nh.loginfo("Opticar ECU connected");
  delay(1);
}

extern "C"
{
  void SysTickIntHandler(void);
}

void loop()
{
  // SysTickIntHandler();
  Led7.off();

  static unsigned long prevControlTime = 0;   // ms
  static unsigned long prevImuTime = 0;       // ms
  static unsigned long prevGpsTime = 0;       // ms
  static unsigned long prevDebugTime = 0;     // ms
  static unsigned long prevHeartbeatTime = 0; // ms
  static bool imuIsInitialized = false;

  static unsigned long ticks = 0;
  ++ticks;
  if (ticks > 80000)
  {
    Led5.toggle();
    ticks = 0;
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
    Led1.toggle();
    prevHeartbeatTime = millis();
  }

  nh.spinOnce();
}

void commandCallback(const geometry_msgs::Twist &twist) {}

void PidCallback(const opticar_msgs::PID &pid) {}

void printDebug()
{
  char buffer[50];
  sprintf(buffer, "Debug chatter");
  nh.loginfo(buffer);
}