#include "../drivers/motors.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "../constants.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"

#ifdef HAVE_UART_PRINTF
#include "../drivers/uartstdio.h"
#endif
#include "../led.h"

// #define VERBOSE

static struct
{
    uint32_t PWM_BASE;
    uint32_t PWM_GEN;
    uint32_t PWM_PIN;
    uint32_t PWM_PIN_BIT;
    uint32_t DIRECTION_BASE;
    uint32_t DIRECTION_PIN;
    int32_t currentSpeedPercentage;
    bool invert;
} MOTORDATA[] = {
                 {PWM1_BASE, PWM_GEN_3, PWM_OUT_7, PWM_OUT_7_BIT, GPIO_PORTD_BASE, GPIO_PIN_7, 0, false}, // VL
                 {PWM1_BASE, PWM_GEN_3, PWM_OUT_6, PWM_OUT_6_BIT, GPIO_PORTD_BASE, GPIO_PIN_6, 0, true}, // VR
                 {PWM0_BASE, PWM_GEN_3, PWM_OUT_7, PWM_OUT_7_BIT, GPIO_PORTC_BASE, GPIO_PIN_6, 0, false}, // HL
                 {PWM0_BASE, PWM_GEN_3, PWM_OUT_6, PWM_OUT_6_BIT, GPIO_PORTB_BASE, GPIO_PIN_3, 0, true}, // HR
};

static struct
{
  uint32_t GPIO_BASE;
  uint32_t GPIO_PIN;
  uint32_t counter;
} ENCODERDATA[] = {
  {GPIO_PORTD_BASE, GPIO_PIN_0, 0}, // VL
  {GPIO_PORTE_BASE, GPIO_PIN_0, 0}, // VR
  {GPIO_PORTD_BASE, GPIO_PIN_1, 0}, // HL
  {GPIO_PORTD_BASE, GPIO_PIN_2, 0}, // HR
};


uint32_t PWM_PERIOD = 0;
uint32_t EMERGENCY_STOP = 0;

#define MAX_MOTOR_SPEED 12

static int32_t clamp(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

// Encoder interrupts
void InterruptEncoder()
{
  for (int i = 0; i < (sizeof(ENCODERDATA) / sizeof(ENCODERDATA[0])); ++i)
  {
    uint32_t intStatus = GPIOIntStatus(ENCODERDATA[i].GPIO_BASE, true);
    if (intStatus & ENCODERDATA[i].GPIO_PIN)
    {
      GPIOIntClear(ENCODERDATA[i].GPIO_BASE, ENCODERDATA[i].GPIO_PIN);
      if (MOTORDATA[i].currentSpeedPercentage < 0)
      {
        --ENCODERDATA[i].counter;
      }
      else
      {
        ++ENCODERDATA[i].counter;
      }
    }
  }
}

bool MOTOR_Init()
{
    // Set the clock to 5 MHz
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

    // Enable the PWM peripherals
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);


    // Setup pin muxing
    ROM_GPIOPinConfigure(GPIO_PC4_M0PWM6); // HL;
    ROM_GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);

    ROM_GPIOPinConfigure(GPIO_PC5_M0PWM7); // HR
    ROM_GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);

    ROM_GPIOPinConfigure(GPIO_PF2_M1PWM6); // VL
    ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    ROM_GPIOPinConfigure(GPIO_PF3_M1PWM7); // VR
    ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;	// unlock the GPIOCR register for port F
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;		// Free up pin 7
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x00;

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);                    // PB3 - DIR_HL
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);       // PC6 - DIR_HR  PC7 - ENABLE_H
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);       // PD6 - DIR_VL  PD7 - DIR_VR
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);                    // PF4 - ENABLE_V


    uint32_t PWMClock = SysCtlClockGet() / 8;//80MHz/16=5MHz
    PWM_PERIOD = (PWMClock / OPTICAR_PWM_FREQUENCY) - 1; // Must fit within 16 bits -> PWM frequency must be at least 320 Hz
#ifdef HAVE_UART_PRINTF
    UARTprintf("Configuring a PWM period of %d cycles using PWM clock of %d kHz\n", (PWM_PERIOD + 1), PWMClock/1000);
#endif

    // To see used Generators visit Datasheet Page 1331 (M0_PWM6 and M0_PWM7 are controlled by Gen3, M1_PWM6 and M1_PWM7 are controlled by Gen3)
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWM_PERIOD);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PWM_PERIOD);

    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); // Enable front driver board, Yan
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7); // Enable rear  driver board, Yan

    MOTOR_SetSpeed(FRONT_LEFT, 0);
    MOTOR_SetSpeed(FRONT_RIGHT, 0);
    MOTOR_SetSpeed(REAR_LEFT, 0);
    MOTOR_SetSpeed(REAR_RIGHT, 0);

    // Initialize encoders
    for (int wheel = 0; wheel < (sizeof(ENCODERDATA) / sizeof(ENCODERDATA[0])); ++wheel)
    {
      GPIOPinTypeGPIOInput(ENCODERDATA[wheel].GPIO_BASE, ENCODERDATA[wheel].GPIO_PIN);
      GPIOPadConfigSet(ENCODERDATA[wheel].GPIO_BASE, ENCODERDATA[wheel].GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
      GPIOIntRegister(ENCODERDATA[wheel].GPIO_BASE, InterruptEncoder);
      GPIOIntTypeSet(ENCODERDATA[wheel].GPIO_BASE, ENCODERDATA[wheel].GPIO_PIN, GPIO_RISING_EDGE);
      GPIOIntEnable(ENCODERDATA[wheel].GPIO_BASE, ENCODERDATA[wheel].GPIO_PIN);
    }

    return true;
}

bool MOTOR_EmergencyStop()
{
    if (EMERGENCY_STOP) return true;

    EMERGENCY_STOP = true;

    MOTOR_SetSpeed(FRONT_LEFT,  0);
    MOTOR_SetSpeed(FRONT_RIGHT, 0);
    MOTOR_SetSpeed(REAR_LEFT,   0);
    MOTOR_SetSpeed(REAR_RIGHT,  0);

    LED_SetSystemLED(SYSTEM_LED_EMERGENCYSTOP, true);

    LED_Set(LED_BLOCK_INDL, LED_AMBER);
    LED_Set(LED_BLOCK_INDR, LED_AMBER);
    LED_Set(LED_BLOCK_REAR, LED_AMBER);

    LED_Set(LED_BLOCK_VL, LED_BLUE);
    LED_Set(LED_BLOCK_VR, LED_BLUE);
    LED_Set(LED_BLOCK_HL, LED_BLUE);
    LED_Set(LED_BLOCK_HR, LED_BLUE);

    return true;
}

bool MOTOR_ReleaseEmergencyStop()
{
    if (!EMERGENCY_STOP) return false;
    EMERGENCY_STOP = false;

    LED_SetSystemLED(SYSTEM_LED_EMERGENCYSTOP, false);

    LED_Set(LED_BLOCK_INDL, LED_GREEN);
    LED_Set(LED_BLOCK_INDR, LED_GREEN);
    LED_Set(LED_BLOCK_REAR, LED_GREEN);

    return false;
}

void MOTOR_SetSpeed(MOTOR_Wheels wheel, int32_t percentage)
{

    const uint32_t DIR_FORWARD = 0;
    const uint32_t DIR_BACKWARD = 1;

    if (EMERGENCY_STOP) return;
    if (wheel > REAR_RIGHT) return;
    percentage = clamp(percentage, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    int direction = percentage > 0 ? DIR_FORWARD : DIR_BACKWARD;
    int absSpeed = percentage > 0 ? percentage : -percentage;

    int pulseWidth = (absSpeed * PWM_PERIOD) / 100;

    MOTORDATA[wheel].currentSpeedPercentage = percentage;

    if (pulseWidth < 1)
    {
        PWMOutputState(MOTORDATA[wheel].PWM_BASE, MOTORDATA[wheel].PWM_PIN_BIT, true);
        PWMPulseWidthSet(MOTORDATA[wheel].PWM_BASE, MOTORDATA[wheel].PWM_PIN, 0);

        switch (wheel)
        {
        case FRONT_LEFT:
            LED_Set(LED_BLOCK_VL, LED_BLUE);
            break;
        case FRONT_RIGHT:
            LED_Set(LED_BLOCK_VR, LED_BLUE);
            break;
        case REAR_LEFT:
            LED_Set(LED_BLOCK_HL, LED_BLUE);
            break;
        case REAR_RIGHT:
            LED_Set(LED_BLOCK_HR, LED_BLUE);
            break;
        }
    }
    else
    {
        switch (wheel)
        {
        case FRONT_LEFT:
            LED_Set(LED_BLOCK_VL, (direction == DIR_FORWARD) ? LED_GREEN : LED_RED);
            break;
        case FRONT_RIGHT:
            LED_Set(LED_BLOCK_VR, (direction == DIR_FORWARD) ? LED_GREEN : LED_RED);
            break;
        case REAR_LEFT:
            LED_Set(LED_BLOCK_HL, (direction == DIR_FORWARD) ? LED_GREEN : LED_RED);
            break;
        case REAR_RIGHT:
            LED_Set(LED_BLOCK_HR, (direction == DIR_FORWARD) ? LED_GREEN : LED_RED);
            break;
        }

        if (MOTORDATA[wheel].invert)
        {
            direction = (direction ? 0 : 1);
        }

        PWMPulseWidthSet(MOTORDATA[wheel].PWM_BASE, MOTORDATA[wheel].PWM_PIN, pulseWidth);
#ifdef VERBOSE
        UARTprintf("Setting a load value of %d for wheel %d\n", pulseWidth, wheel);
#endif
        PWMOutputState(MOTORDATA[wheel].PWM_BASE, MOTORDATA[wheel].PWM_PIN_BIT, true);
        GPIOPinWrite(MOTORDATA[wheel].DIRECTION_BASE, MOTORDATA[wheel].DIRECTION_PIN, direction * MOTORDATA[wheel].DIRECTION_PIN);
    }
}

uint32_t MOTOR_GetEncoderCounter(MOTOR_Wheels wheel)
{
  return ENCODERDATA[wheel].counter;
}
