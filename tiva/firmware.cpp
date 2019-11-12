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
#include <Encoder.h>
#include <Imu.h>
#include <Motor.h>
#include <PID.h>
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

// Controller for front left motor
PID PidMotor1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// Controller for front right motor
PID PidMotor2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// Controller for rear left motor
PID PidMotor3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// Controller for rear right motor
PID PidMotor4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Motor driver for front left motor
Controller Motor1(0, DemoLED::LED_BLOCK_VL);
// Motor driver for front right motor
Controller Motor2(1, DemoLED::LED_BLOCK_VR);
// Motor driver for rear left motor
Controller Motor3(2, DemoLED::LED_BLOCK_HL);
// Motor driver for rear right motor
Controller Motor4(3, DemoLED::LED_BLOCK_HR);

// Encoder for front left motor
SingleEncoder EncoderMotor1(ENCODER_PIN1_FRONT_LEFT, OPTICAR_IMPULSES_PER_REVOLUTION);
// Encoder for front right motor
SingleEncoder EncoderMotor2(ENCODER_PIN1_FRONT_RIGHT, OPTICAR_IMPULSES_PER_REVOLUTION);
// Encoder for rear left motor
SingleEncoder EncoderMotor3(ENCODER_PIN1_REAR_LEFT, OPTICAR_IMPULSES_PER_REVOLUTION);
// Encoder for rear right motor
SingleEncoder EncoderMotor4(ENCODER_PIN1_REAR_RIGHT, OPTICAR_IMPULSES_PER_REVOLUTION);

unsigned long g_PrevCommandTime = 0;  // ms

float g_RequestedLinearVelocityX = 0;   // m/s
float g_RequestedLinearVelocityY = 0;   // m/s
float g_RequestedAngularVelocityZ = 0;  // rad/s

// Callbacks for the message subscribers
void commandCallback(const geometry_msgs::Twist &twist);
void pidCallback(const opticar_msgs::PID &pid);

// Other forward declarations
void printDebug();
void publishIMU();
void moveBase();
void stopBase();

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

  // Initialize motors
  Motor1.init(nh);
  Motor2.init(nh);
  Motor3.init(nh);
  Motor4.init(nh);

  // Enable motor drivers
  pinMode(MOTOR_PIN_ENABLE_FRONT, OUTPUT);
  digitalWrite(MOTOR_PIN_ENABLE_FRONT, 1);
  pinMode(MOTOR_PIN_ENABLE_REAR, OUTPUT);
  digitalWrite(MOTOR_PIN_ENABLE_REAR, 1);

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

  // Apply drive command with the given update rate
  if ((millis() - prevControlTime) >= (1000 / COMMAND_RATE))
  {
    moveBase();
    prevControlTime = millis();
  }

  // Engage emergency stop if no command was received for a given time
  if ((millis() - g_PrevCommandTime) >= EMERGENCY_STOP_TIMEOUT)
  {
    stopBase();
  }

  // Publish IMU data with the given update rate
  if ((millis() - prevImuTime) >= (1000 / IMU_PUBLISH_RATE))
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

  // Output some debug data if enabled
  if (DEBUG)
  {
    if ((millis() - prevDebugTime) >= (1000 / DEBUG_RATE))
    {
      printDebug();
      prevDebugTime = millis();
    }
  }

  if ((millis() - prevHeartbeatTime) >= (1000 / HEARTBEAT_RATE))
  {
    LedHeartbeat.toggle();
    prevHeartbeatTime = millis();
  }

  nh.spinOnce();
}

void commandCallback(const geometry_msgs::Twist &twist)
{
  g_RequestedLinearVelocityX = twist.linear.x;
  g_RequestedLinearVelocityY = twist.linear.y;
  g_RequestedAngularVelocityZ = twist.angular.z;

  g_PrevCommandTime = millis();
}

void pidCallback(const opticar_msgs::PID &pid)
{
  // TODO
}

void moveBase()
{
  LedEmergencyStop.off();
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_INDL, DemoLED::LED_GREEN);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_INDR, DemoLED::LED_GREEN);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_REAR, DemoLED::LED_GREEN);

  // TODO
}

void stopBase()
{
  LedEmergencyStop.on();
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_INDL, DemoLED::LED_BLUE);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_INDR, DemoLED::LED_BLUE);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_REAR, DemoLED::LED_RED);

  g_RequestedLinearVelocityX = 0;
  g_RequestedLinearVelocityY = 0;
  g_RequestedAngularVelocityZ = 0;
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

  sprintf(buffer, "Encoder FrontLeft  : %ld", EncoderMotor1.read());
  nh.loginfo(buffer);
  sprintf(buffer, "Encoder FrontRight : %ld", EncoderMotor2.read());
  nh.loginfo(buffer);
  sprintf(buffer, "Encoder RearLeft   : %ld", EncoderMotor3.read());
  nh.loginfo(buffer);
  sprintf(buffer, "Encoder RearRight  : %ld", EncoderMotor4.read());
  nh.loginfo(buffer);
}