#ifndef OPTICAR_CONFIG_H
#define OPTICAR_CONFIG_H

#define OPTICAR_NG

#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

#define DEBUG 1

#define K_P 0.6  // proportional constant
#define K_I 0.3  // integral constant
#define K_D 0.5  // derivative constant

#define IMU_PUBLISH_RATE 20  // Hz
#define GPS_PUBLISH_RATE 5   // Hz
#define COMMAND_RATE 20      // Hz
#define DEBUG_RATE 1         // Hz
#define HEARTBEAT_RATE 2     // Hz

#define EMERGENCY_STOP_TIMEOUT 400  // ms

#define PWM_MAX 20.0f  // Percentage
#define PWM_MIN -PWM_MAX

#define OPTICAR_PWM_FREQUENCY 440  // Hz

#define OPTICAR_MAX_RPM 394
#define OPTICAR_IMPULSES_PER_REVOLUTION 87.3
#define OPTICAR_WHEEL_DIAMETER 0.203  // m
#define OPTICAR_BASE_WIDTH 0.62       // m
#define OPTICAR_BASE_LENGTH 0.84      // m

static struct MotorConfigData
{
  uint32_t PWM_BASE;
  uint32_t PWM_GEN;
  uint32_t PWM_PIN;
  uint32_t PWM_PIN_BIT;
  uint32_t CFG_TYPE_PIN;
  uint32_t OUTPUT_BASE;
  uint32_t OUTPUT_PIN;
  uint32_t DIRECTION_BASE;
  uint32_t DIRECTION_PIN;
  bool invert;
} MOTORCONFIGDATA[] = {
  { PWM1_BASE, PWM_GEN_3, PWM_OUT_7, PWM_OUT_7_BIT, GPIO_PF2_M1PWM6, GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PORTB_BASE,
    GPIO_PIN_3, true },  // VL
  { PWM1_BASE, PWM_GEN_3, PWM_OUT_6, PWM_OUT_6_BIT, GPIO_PF3_M1PWM7, GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PORTC_BASE,
    GPIO_PIN_6, false },  // VR
  { PWM0_BASE, PWM_GEN_3, PWM_OUT_7, PWM_OUT_7_BIT, GPIO_PC4_M0PWM6, GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PORTD_BASE,
    GPIO_PIN_6, true },  // HL
  { PWM0_BASE, PWM_GEN_3, PWM_OUT_6, PWM_OUT_6_BIT, GPIO_PC5_M0PWM7, GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PORTD_BASE,
    GPIO_PIN_7, false },  // HR
};

static int MOTORDIRECTIONPINS[] = { 38, 35, 33, 32 };

#define MOTOR_PIN_NSLEEP 34
#define MOTOR_PIN_NFAULT 31

#define ENCODER_PIN1_FRONT_LEFT 29
#define ENCODER_PIN2_FRONT_LEFT 28

#define ENCODER_PIN1_FRONT_RIGHT 27
#define ENCODER_PIN2_FRONT_RIGHT 26

#define ENCODER_PIN1_REAR_LEFT 25
#define ENCODER_PIN2_REAR_LEFT 8

#define ENCODER_PIN1_REAR_RIGHT 2
#define ENCODER_PIN2_REAR_RIGHT 18

#define g_CountFrontLeft 5
#define g_CountFrontRight 5
#define g_CountRearLeft 5
#define g_CountRearRight 5
#define g_CountIndiLeft 3
#define g_CountIndiRight 3
#define g_CountRear 12

#endif