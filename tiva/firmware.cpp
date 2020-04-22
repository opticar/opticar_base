// This firmware was adapted from the linorobot teensy firmware
// Available at
// https://github.com/linorobot/linorobot/blob/master/teensy/firmware/src/firmware.ino
//
// To flash, use catkin build --no-deps opticar_base --make-args opticar_base_tiva_flash
// To test, run a rosserial node: rosrun rosserial_python serial_node.py _port:=/dev/tiva _baud:=115200

#include <Energia.h>

#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "tivac_hardware.h"

// Various configuration settings
#include OPTICAR_CFG

// Comment these defines to disable features
#define USE_MOTORS
// #define USE_ENCODERS // Disabled for now, needs hardware debugging
#define USE_SYSTEM_LEDS
#define USE_DEMO_LEDS
#define USE_IMU
#define USE_GPS

// Various drivers
#include <DemoLED.h>
#include <Encoder.h>
#include <GPS.h>
#include <Imu.h>
#include <Kinematics.h>
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
// header file for GPS data
#include <sensor_msgs/NavSatFix.h>

// Controller for front left motor
PID PidMotor1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// Controller for front right motor
PID PidMotor2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// Controller for rear left motor
PID PidMotor3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// Controller for rear right motor
PID PidMotor4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

#ifdef USE_MOTORS
// Motor driver for front left motor
Controller Motor1(0, DemoLED::LED_BLOCK_VL);
// Motor driver for front right motor
Controller Motor2(1, DemoLED::LED_BLOCK_VR);
// Motor driver for rear left motor
Controller Motor3(2, DemoLED::LED_BLOCK_HL);
// Motor driver for rear right motor
Controller Motor4(3, DemoLED::LED_BLOCK_HR);
#endif

#ifdef USE_ENCODERS
// Encoder for front left motor
SingleEncoder EncoderMotor1(ENCODER_PIN1_FRONT_LEFT, OPTICAR_IMPULSES_PER_REVOLUTION);
// Encoder for front right motor
SingleEncoder EncoderMotor2(ENCODER_PIN1_FRONT_RIGHT, OPTICAR_IMPULSES_PER_REVOLUTION);
// Encoder for rear left motor
SingleEncoder EncoderMotor3(ENCODER_PIN1_REAR_LEFT, OPTICAR_IMPULSES_PER_REVOLUTION);
// Encoder for rear right motor
SingleEncoder EncoderMotor4(ENCODER_PIN1_REAR_RIGHT, OPTICAR_IMPULSES_PER_REVOLUTION);
#endif

Kinematics BaseKinematics(Kinematics::MECANUM, OPTICAR_MAX_RPM, OPTICAR_WHEEL_DIAMETER, OPTICAR_BASE_LENGTH,
                          OPTICAR_BASE_WIDTH);

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

sensor_msgs::NavSatFix navSatFix;
ros::Publisher rawNavSatFixPub("raw_gps", &navSatFix);

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

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

#ifdef USE_SYSTEM_LEDS
  // Initialize on-board LEDs
  SystemLED::init();
#endif
  // Switch on heartbeat LED to indicate we are waiting for a connection
  LedHeartbeat.on();

#ifdef USE_DEMO_LEDS
  // Initialize demonstrator LEDs
  DemoLed.init();
#endif

  // Initialize communication
  nh.initNode();

#ifdef USE_MOTORS
  // Initialize motors
  Motor1.init(nh);
  Motor2.init(nh);
  Motor3.init(nh);
  Motor4.init(nh);
#endif

  // Enable motor drivers
#ifdef OPTICAR_V1
  pinMode(MOTOR_PIN_ENABLE_FRONT, OUTPUT);
  digitalWrite(MOTOR_PIN_ENABLE_FRONT, 1);
  pinMode(MOTOR_PIN_ENABLE_REAR, OUTPUT);
  digitalWrite(MOTOR_PIN_ENABLE_REAR, 1);
#else
  pinMode(MOTOR_PIN_NSLEEP, OUTPUT);
  digitalWrite(MOTOR_PIN_NSLEEP, 1);
#endif

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
  nh.advertise(rawNavSatFixPub);

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
#ifdef USE_IMU
      imuIsInitialized = initIMU();

      if (imuIsInitialized)
      {
        nh.loginfo("IMU initialized");
      }
      else
      {
        nh.logfatal("IMU failed to initialize. Check IMU connection.");
      }
#endif
    }
    else
    {
      publishIMU();
    }

    prevImuTime = millis();
  }

#ifdef USE_GPS
  // Always update GPS internal data
  BaseGPS.step();
#endif
  // Publish GPS data with the given update rate
  if ((millis() - prevGpsTime) >= (1000 / GPS_PUBLISH_RATE))
  {
#ifdef USE_GPS
    BaseGPS.updateMessage(navSatFix);
#endif
    rawNavSatFixPub.publish(&navSatFix);

    prevGpsTime = millis();
  }

  // Output some debug data if enabled
  if (DEBUG_OUTPUT)
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

  LedEmergencyStop.off();
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_INDL, DemoLED::LED_GREEN);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_INDR, DemoLED::LED_GREEN);
  DemoLed.setBlockColor(DemoLED::LED_BLOCK_REAR, DemoLED::LED_GREEN);
}

void pidCallback(const opticar_msgs::PID &pid)
{
  PidMotor1.updateConstants(pid.p, pid.i, pid.d);
  PidMotor2.updateConstants(pid.p, pid.i, pid.d);
  PidMotor3.updateConstants(pid.p, pid.i, pid.d);
  PidMotor4.updateConstants(pid.p, pid.i, pid.d);
}

void moveBase()
{
  Kinematics::rpm reqRPM =
      BaseKinematics.getRPM(g_RequestedLinearVelocityX, g_RequestedLinearVelocityY, g_RequestedAngularVelocityZ);

#ifdef USE_ENCODERS
  int currentRPM1 = EncoderMotor1.getRPM();
  int currentRPM2 = EncoderMotor2.getRPM();
  int currentRPM3 = EncoderMotor3.getRPM();
  int currentRPM4 = EncoderMotor4.getRPM();
#else
  int currentRPM1 = 0;
  int currentRPM2 = 0;
  int currentRPM3 = 0;
  int currentRPM4 = 0;
#endif

#ifdef USE_MOTORS
  // Adjust RPM for motor direction
  if (Motor1.getCurrentDirection() == Controller::DIR_BACKWARD)
    currentRPM1 = -currentRPM1;
  if (Motor2.getCurrentDirection() == Controller::DIR_BACKWARD)
    currentRPM2 = -currentRPM2;
  if (Motor3.getCurrentDirection() == Controller::DIR_BACKWARD)
    currentRPM3 = -currentRPM3;
  if (Motor4.getCurrentDirection() == Controller::DIR_BACKWARD)
    currentRPM4 = -currentRPM4;

  // Update motors via controllers
  float targetSpeedMotor1 = PidMotor1.compute(reqRPM.motor1, currentRPM1);
  float targetSpeedMotor2 = PidMotor2.compute(reqRPM.motor2, currentRPM1);
  float targetSpeedMotor3 = PidMotor3.compute(reqRPM.motor3, currentRPM1);
  float targetSpeedMotor4 = PidMotor4.compute(reqRPM.motor4, currentRPM1);

  Motor1.spin(targetSpeedMotor1);
  Motor2.spin(targetSpeedMotor2);
  Motor3.spin(targetSpeedMotor3);
  Motor4.spin(targetSpeedMotor4);
#endif

  Kinematics::velocities currentVel = BaseKinematics.getVelocities(currentRPM1, currentRPM2, currentRPM3, currentRPM4);

  rawVelMsg.linear_x = currentVel.linear_x;
  rawVelMsg.linear_y = currentVel.linear_y;
  rawVelMsg.rotation_z = currentVel.angular_z;

  rawVelPub.publish(&rawVelMsg);
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
#ifdef USE_IMU
  rawImuMsg.acceleration = readAccelerometer();
  rawImuMsg.rotation = readGyroscope();
  rawImuMsg.magnetic = readMagnetometer();
#endif

  rawImuPub.publish(&rawImuMsg);
}

void printDebug()
{
  char buffer[50];

#ifdef USE_ENCODERS
  snprintf(buffer, 50, "Encoder FrontLeft  : %ld", EncoderMotor1.read());
  nh.loginfo(buffer);
  snprintf(buffer, 50, "Encoder FrontRight : %ld", EncoderMotor2.read());
  nh.loginfo(buffer);
  snprintf(buffer, 50, "Encoder RearLeft   : %ld", EncoderMotor3.read());
  nh.loginfo(buffer);
  snprintf(buffer, 50, "Encoder RearRight  : %ld", EncoderMotor4.read());
  nh.loginfo(buffer);
#endif
}