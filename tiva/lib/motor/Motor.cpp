#include "Motor.h"

#include "driverlib/sysctl.h"

uint32_t Controller::pwmPeriod = 0;

Controller::Controller(int configDataIndex, const DemoLED::LEDBlock ledBlock)
  : configDataIndex(configDataIndex), initialized(false), ledBlock(ledBlock), currentLogicalDirection(0)
{
}

void Controller::init(ros::NodeHandle& nh)
{
  static bool GlobalPwmConfigured = false;

  // Setup global PWM clock to 5 MHz (with system clock at 80 MHz) and enable peripherals
  if (!GlobalPwmConfigured)
  {
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    uint32_t pwmFrequency = SysCtlClockGet() / 8;  // We use double the original frequency for calculation since we
                                                   // generate centered pulses in up-down mode
    pwmPeriod = (pwmFrequency / OPTICAR_PWM_FREQUENCY) - 1;

    // PWM uses 16bit counters internally, so check if we exceed that range
    if (pwmPeriod > 65535)
    {
      char buffer[50];
      sprintf(buffer, "Cannot set PWM frequency %d due to PWM period overflow", OPTICAR_PWM_FREQUENCY);
      nh.logfatal(buffer);
      return;
    }

    // Setup PWM generators
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN);
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, pwmPeriod);
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, pwmPeriod);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);

#if DEBUG
    char buffer[50];
    sprintf(buffer, "Configuring PWM period of %d", pwmPeriod + 1);
    nh.loginfo(buffer);
#endif

    GlobalPwmConfigured = true;
  }

  // Setup pin muxing
  ROM_GPIOPinConfigure(MOTORCONFIGDATA[configDataIndex].CFG_TYPE_PIN);
  ROM_GPIOPinTypePWM(MOTORCONFIGDATA[configDataIndex].OUTPUT_BASE, MOTORCONFIGDATA[configDataIndex].OUTPUT_PIN);

  ROM_GPIOPinTypeGPIOOutput(MOTORCONFIGDATA[configDataIndex].DIRECTION_BASE,
                            MOTORCONFIGDATA[configDataIndex].DIRECTION_PIN);

  initialized = true;

  spin(0.0f);
}

void Controller::spin(float pwmPercentage)
{
  if (!initialized)
    return;

  int direction = (pwmPercentage > 0) ? DIR_FORWARD : DIR_BACKWARD;
  currentLogicalDirection = direction;

  if (MOTORCONFIGDATA[configDataIndex].invert)
  {
    direction = (direction ? 0 : 1);
  }

  float absSpeed = fabs(pwmPercentage);
  uint32_t pulseWidth = (pwmPeriod * absSpeed) / 100;

  PWMPulseWidthSet(MOTORCONFIGDATA[configDataIndex].PWM_BASE, MOTORCONFIGDATA[configDataIndex].PWM_PIN, pulseWidth);
  PWMOutputState(MOTORCONFIGDATA[configDataIndex].PWM_BASE, MOTORCONFIGDATA[configDataIndex].PWM_PIN_BIT, true);
  GPIOPinWrite(MOTORCONFIGDATA[configDataIndex].DIRECTION_BASE, MOTORCONFIGDATA[configDataIndex].DIRECTION_PIN,
               direction);

  if (pulseWidth < 1)
  {
    DemoLed.setBlockColor(ledBlock, DemoLED::LED_BLUE);
  }
  else
  {
    float pwmRatio = absSpeed / PWM_MAX;
    if (currentLogicalDirection == DIR_FORWARD)
    {
      DemoLed.setBlockRaw(ledBlock, 0, 255 * pwmRatio, 0);
    }
    else
    {
      DemoLed.setBlockRaw(ledBlock, 255 * pwmRatio, 0, 0);
    }
  }
}

int Controller::getCurrentDirection()
{
  return currentLogicalDirection;
}
