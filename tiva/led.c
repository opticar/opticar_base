#include "led.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"


#include "drivers/SPI_uDMA_drv.h"
#include "drivers/WS2812_drv.h"
#ifdef HAVE_UART_PRINTF
#include "drivers/uartstdio.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define CHANNEL_RED 1
#define CHANNEL_GREEN 0
#define CHANNEL_BLUE 2

#define LED_OFF 0
#define LED_HALF 127
#define LED_ON 255

uint8_t g_ShowLEDs[NUM_SHOW_LEDS][3]; // GRB
uint8_t g_SPIOut[NUM_SHOW_LEDS][WS2812_SPI_BYTE_PER_CLR * WS2812_SPI_BIT_WIDTH];
uint8_t g_SPIDone;

bool LED_Init()
{
    g_SPIDone = 0;

    InitSPITransfer((uint8_t*)g_SPIOut, sizeof(g_SPIOut), &g_SPIDone);

    LED_SetLEDBlockColor(g_StartFrontLeft, g_CountFrontLeft, LED_BLACKOUT);
    LED_SetLEDBlockColor(g_StartFrontRight, g_CountFrontRight, LED_BLACKOUT);
    LED_SetLEDBlockColor(g_StartRearLeft, g_CountRearLeft, LED_BLACKOUT);
    LED_SetLEDBlockColor(g_StartRearRight, g_CountRearRight, LED_BLACKOUT);
    LED_SetLEDBlockColor(g_StartIndiLeft, g_CountIndiLeft, LED_BLACKOUT);
    LED_SetLEDBlockColor(g_StartIndiRight, g_CountIndiRight, LED_BLACKOUT);
    LED_SetLEDBlockColor(g_StartRear, g_CountRear, LED_BLACKOUT);
    
    // Initialize board LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);// LED7
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);// LED6
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);// LED5
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);// LED4
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);// LED3
    // LED2 not available on Revision 2
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);// LED1

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0x0);// LED7
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);// LED6
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);// LED5
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0x0);// LED4
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x0);// LED3
    // LED2
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);// LED1

    return true;
}

void LED_SetSystemLED(SystemLED led, bool on)
{
    switch (led)
    {
    case SYSTEM_LED_READY:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, (on ? GPIO_PIN_2 : 0x0));
        break;
    case SYSTEM_LED_EMERGENCYSTOP:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, (on ? GPIO_PIN_3 : 0x0));
        break;
    case SYSTEM_LED_HEARTBEAT:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, (on ? GPIO_PIN_4 : 0x0));
        break;
    case SYSTEM_LED_4:
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, (on ? GPIO_PIN_6 : 0x0));
        break;
    case SYSTEM_LED_3:
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, (on ? GPIO_PIN_7 : 0x0));
        break;
    case SYSTEM_LED_2:
        break;
    case SYSTEM_LED_1:
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, (on ? GPIO_PIN_0 : 0x0));
        break;
    default:
#ifdef HAVE_UART_PRINTF
        UARTprintf("Unknown system LED %d\n", led);
#endif
        break;
    }
}

void LED_SetLEDBlock(int start, int count, uint8_t red, uint8_t green, uint8_t blue)
{
    int i = 0;
    start += LED_OFFSET;

    for (i = 0; i < count; ++i)
    {
        g_ShowLEDs[start + i][CHANNEL_RED] = red;
        g_ShowLEDs[start + i][CHANNEL_GREEN] = green;
        g_ShowLEDs[start + i][CHANNEL_BLUE] = blue;

        WSGRBtoSPI(g_SPIOut[start + i],
                   g_ShowLEDs[start + i][0],
                   g_ShowLEDs[start + i][1],
                   g_ShowLEDs[start + i][2]);
    }
}

void LED_SetLEDBlockColor(int start, int count, LEDColor color)
{
    uint8_t red = LED_OFF;
    uint8_t green = LED_OFF;
    uint8_t blue = LED_OFF;

    switch(color)
    {
    case LED_RED:
        red = LED_ON;
        break;
    case LED_YELLOW:
        red = LED_ON;
        green = LED_ON;
        break;
    case LED_GREEN:
        green = LED_ON;
        break;
    case LED_CYAN:
        green = LED_ON;
        blue = LED_ON;
        break;
    case LED_BLUE:
        blue = LED_ON;
        break;
    case LED_MAGENTA:
        blue = LED_ON;
        red = LED_ON;
        break;
    case LED_AMBER:
        red = LED_ON;
        green = LED_HALF;
        break;
    case LED_WHITE:
        red = LED_ON;
        green = LED_ON;
        blue = LED_ON;
        break;
    case LED_BLACKOUT:
	break;
    }
    LED_SetLEDBlock(start, count, red, green, blue);
}

void LED_Set(LEDBlock block, LEDColor color)
{
    switch (block)
    {
    case LED_BLOCK_VL:
        LED_SetLEDBlockColor(g_StartFrontLeft, g_CountFrontLeft, color);
        break;
    case LED_BLOCK_VR:
        LED_SetLEDBlockColor(g_StartFrontRight, g_CountFrontRight, color);
        break;
    case LED_BLOCK_HL:
        LED_SetLEDBlockColor(g_StartRearLeft, g_CountRearLeft, color);
        break;
    case LED_BLOCK_HR:
        LED_SetLEDBlockColor(g_StartRearRight, g_CountRearRight, color);
        break;
    case LED_BLOCK_INDL:
        LED_SetLEDBlockColor(g_StartIndiLeft, g_CountIndiLeft, color);
        break;
    case LED_BLOCK_INDR:
        LED_SetLEDBlockColor(g_StartIndiRight, g_CountIndiRight, color);
        break;
    case LED_BLOCK_REAR:
        LED_SetLEDBlockColor(g_StartRear, g_CountRear, color);
        break;
    }
}

#ifdef __cplusplus
};
#endif
