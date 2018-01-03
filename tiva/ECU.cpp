#include <stdbool.h>
#include <stdint.h>
// TivaC specific includes
extern "C"
{
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/cpu.h>
  #define restrict
  #include <utils/ustdlib.h>
}
// ROS includes
#include <ros.h>
#include <ros/time.h>
#include <opticar_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <opticar_msgs/PID.h>
#include <geometry_msgs/Vector3.h>
#include <opticar_msgs/IMU.h>

// ECU hardware includes
#include "led.h"
#include "drivers/motors.h"
#include "drivers/GPS.h"
#include "drivers/i2c.h"
#include "drivers/mpu9250.h"
#include "constants.h"
#include "kinematic.h"
#include "config.h"

void CB_Command(const geometry_msgs::Twist& msg);
void CB_PID(const opticar_msgs::PID& msg);
void BASE_Drive(Kinematic& kh, ros::NodeHandle& nh);
void BASE_Stop();

// ROS nodehandle
ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> subCommand("cmd_vel", CB_Command);
ros::Subscriber<opticar_msgs::PID> subPID("pid", CB_PID);

opticar_msgs::IMU rawIMUMsg;
ros::Publisher pubRawIMU("raw_imu", &rawIMUMsg);
opticar_msgs::Velocities rawVelMsg;
ros::Publisher pubRawVel("raw_vel", &rawVelMsg);

double gTimePrevCmd = 0.0;

double gCmdVelX = 0.0;
double gCmdVelY = 0.0;
double gCmdRotZ = 0.0;

#define DEG_TO_RAD 0.01745329252
#define GRAVITATIONAL_CONSTANT 9.81

int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  
  // Enable GPIO ports
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  
  // Initialize configuration store (EEPROM)
  bool Configok = CFG_Init();
  
  // Initialize LED subsystem
  bool LEDok = LED_Init();
  
  // Clear values as calibration will be handled by the node graph
  g_Cfg.m_AcceleratorCalibX = 0;
  g_Cfg.m_AcceleratorCalibY = 0;
  g_Cfg.m_AcceleratorCalibZ = 0;
  
  LED_Set(LED_BLOCK_VL, LED_BLACKOUT);
  LED_Set(LED_BLOCK_VR, LED_BLACKOUT);
  LED_Set(LED_BLOCK_HL, LED_BLACKOUT);
  LED_Set(LED_BLOCK_HR, LED_BLACKOUT);
  LED_Set(LED_BLOCK_INDL, LED_BLACKOUT);
  LED_Set(LED_BLOCK_INDR, LED_BLACKOUT);
  LED_Set(LED_BLOCK_REAR, LED_BLACKOUT);
  
  // Initialize motor control
  bool Motorok = MOTOR_Init();
  
  // Initialize GPS
  bool GPSok = GPS_Init();
  
  // Initialize I2C
  bool I2Cok = I2C_Init();
  
  // Initialize IMU
  bool IMUok = false;
  MPU9250 mpu;
  uint8_t mpuWhoAmI = 0;
  
  if (I2Cok && Configok)
  {
    mpuWhoAmI = I2C_Read(MPU_I2C_ADDRESS, MPU_REG_WHOAMI);
    if (mpuWhoAmI != MPU_VAL_WHOAMI)
    {
      IMUok = false;
    }
    else
    {
      IMUok = MPU_Init(&mpu);
    }
  }
  
  // Prepare kinematics handler
  Kinematic kh(OPTICAR_MAX_RPM, OPTICAR_WHEEL_DIAMETER, OPTICAR_BASE_WIDTH, OPTICAR_BASE_LENGTH);
  
  // ROS nodehandle initialization and topic registration
  nh.initNode();
  
  nh.subscribe(subCommand);  
  nh.subscribe(subPID);
  
  nh.advertise(pubRawIMU);  
  nh.advertise(pubRawVel);
  
  while(!nh.connected())
  {
    nh.spinOnce();
  }
  
  // Log status
  if(!Configok)
  {
    nh.logerror("Configuration initialization failed");
  }
  
  if(!LEDok)
  {
    nh.logerror("LED initialization failed");
  }  
  
  if (!Motorok)
  {
    nh.logerror("Motor initialization failed");
  }
  
  if (!GPSok)
  {
    nh.logerror("GPS initialization failed");
  }
  
  if (!I2Cok)
  {
    nh.logerror("I2C initialization failed");
  }
  
  if (!IMUok)
  {
    nh.logerror("IMU initialization failed");
  }
  {
    char buffer[50];
    usprintf(buffer, "MPU whoami: %x", mpuWhoAmI);
    nh.loginfo(buffer);
    usprintf(buffer, "MAG whoami: %x", mpu.m_MagWhoAmI);
    nh.loginfo(buffer);
  }
    
  // Everything is set up, start working
  nh.loginfo("Opticar ECU ready");
  LED_SetSystemLED(SYSTEM_LED_READY, true);
  
  LED_Set(LED_BLOCK_VL, LED_BLUE);
  LED_Set(LED_BLOCK_VR, LED_BLUE);
  LED_Set(LED_BLOCK_HL, LED_BLUE);
  LED_Set(LED_BLOCK_HR, LED_BLUE);
  LED_Set(LED_BLOCK_INDL, LED_BLUE);
  LED_Set(LED_BLOCK_INDR, LED_BLUE);
  LED_Set(LED_BLOCK_REAR, LED_BLUE);
  
  double timePrevBeat = 0.0;
  bool beatState = false;

  while (1)
  {
    static double timePrevCtl = 0.0;
    static double timePrevIMU = 0.0;
    static double timePrevDbg = 0.0;
    static double timePrevVel = 0.0;
    
    // Heartbeat LED
    if ((nh.now().toSec() - timePrevBeat) >= 1.0)
    {
      beatState = !beatState;
      LED_SetSystemLED(SYSTEM_LED_HEARTBEAT, beatState);
      timePrevBeat = nh.now().toSec();
    }
    
    // Perform timeout handling if no commands have been received
    if ((nh.now().toSec() - gTimePrevCmd) > OPTICAR_CMD_TIMEOUT)
    {
      // stop movement
      BASE_Stop();
    } else 
    {
      // Update motor control
      if ((nh.now().toSec() - timePrevCtl) >= (1.0 / OPTICAR_CMD_RATE))
      {
        // perform movement
        BASE_Drive(kh, nh);
        timePrevCtl = nh.now().toSec();
      }
  }
    
    // Publish velocities
    if ((nh.now().toSec() - timePrevVel) >= (1.0 / OPTICAR_ODO_RATE))
    {
      rawVelMsg.linear_x = 0;
      rawVelMsg.linear_y = 0;
      rawVelMsg.rotation_z = 0;
      
      pubRawVel.publish(&rawVelMsg);
      
      timePrevVel = nh.now().toSec();
    }
    
    // Publish IMU data
    if ((nh.now().toSec() - timePrevIMU) >= (1.0 / OPTICAR_IMU_RATE))
    {
      if (IMUok)
      {
        if (MPU_DataReady())
        {
          MPU_GetData(&mpu);
        }
        
        rawIMUMsg.acceleration.x = mpu.m_AccelInEgoFrame.m_Axis.m_X;
        rawIMUMsg.acceleration.y = mpu.m_AccelInEgoFrame.m_Axis.m_Y;
        rawIMUMsg.acceleration.z = mpu.m_AccelInEgoFrame.m_Axis.m_Z;
        
        rawIMUMsg.rotation.x = mpu.m_GyroRates.m_Angles.m_AX * DEG_TO_RAD;
        rawIMUMsg.rotation.y = mpu.m_GyroRates.m_Angles.m_AY * DEG_TO_RAD;
        rawIMUMsg.rotation.z = mpu.m_GyroRates.m_Angles.m_AZ * DEG_TO_RAD;
        
        rawIMUMsg.magnetic.x = mpu.m_MagInEgoFrame.m_Axis.m_X;
        rawIMUMsg.magnetic.y = mpu.m_MagInEgoFrame.m_Axis.m_Y;
        rawIMUMsg.magnetic.z = mpu.m_MagInEgoFrame.m_Axis.m_Z;
      }
      else
      {
	      rawIMUMsg.acceleration.x = 0;
        rawIMUMsg.acceleration.y = 0;
        rawIMUMsg.acceleration.z = 0;
      
        rawIMUMsg.rotation.x = 0;
        rawIMUMsg.rotation.y = 0;
        rawIMUMsg.rotation.z = 0;
      
        rawIMUMsg.magnetic.x = 0;
        rawIMUMsg.magnetic.y = 0;
        rawIMUMsg.magnetic.z = 0;
      }
      
      pubRawIMU.publish(&rawIMUMsg);
      
      timePrevIMU = nh.now().toSec();
    }
    
    // Publish debug info
    if (true)
    {
      if ((nh.now().toSec() - timePrevDbg) >= (1.0 / OPTICAR_DBG_RATE))
      {
        nh.logdebug("DBG");
        timePrevDbg = nh.now().toSec();
      }
    }
    
    // Handle all communications and callbacks.
    nh.spinOnce();
  }
}

void CB_Command(const geometry_msgs::Twist& msg)
{
  gCmdVelX = msg.linear.x;
  gCmdVelY = msg.linear.y;
  gCmdRotZ = msg.angular.z;
  
  gTimePrevCmd = nh.now().toSec();
}

void CB_PID(const opticar_msgs::PID& msg)
{
}

void BASE_Drive(Kinematic& kh, ros::NodeHandle& nh)
{
  MOTOR_ReleaseEmergencyStop();
  
  Kinematic::InputSteering input;
  Kinematic::OutputSpeeds motorSpeeds;
  
  input.velX = gCmdVelX;
  input.velY = gCmdVelY;
  input.rotZ = gCmdRotZ;
  
  motorSpeeds = kh.calculateSpeeds(input);
  
  MOTOR_SetSpeed(FRONT_LEFT,  motorSpeeds.percFrontLeft );
  MOTOR_SetSpeed(FRONT_RIGHT, motorSpeeds.percFrontRight);
  MOTOR_SetSpeed(REAR_LEFT,   motorSpeeds.percRearLeft  );
  MOTOR_SetSpeed(REAR_RIGHT,  motorSpeeds.percRearRight );
  
  static float timePrevMsg = 0;
	if ((nh.now().toSec() - timePrevMsg) > 1)
	{
    char buffer[50];
    usprintf(buffer, "front left perc: %d", (int)(motorSpeeds.percFrontLeft * 100));
    nh.loginfo(buffer);
    
    usprintf(buffer, "dbg1 x 100   %d", (int)(motorSpeeds.dbg1 * 100));
    nh.loginfo(buffer);
    usprintf(buffer, "dbg2 x 10000 %d", (int)(motorSpeeds.dbg2 * 10000));
    nh.loginfo(buffer);
    usprintf(buffer, "dbg3 x 10000 %d", (int)(motorSpeeds.dbg3 * 10000));
    nh.loginfo(buffer);
	}	
}

void BASE_Stop()
{
  gCmdVelX = 0;
  gCmdVelY = 0;
  gCmdRotZ = 0;
  
  MOTOR_SetSpeed(FRONT_LEFT,  0);
  MOTOR_SetSpeed(FRONT_RIGHT, 0);
  MOTOR_SetSpeed(REAR_LEFT,   0);
  MOTOR_SetSpeed(REAR_RIGHT,  0);
  
  MOTOR_EmergencyStop();
}
