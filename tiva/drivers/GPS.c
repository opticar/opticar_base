#include "../drivers/GPS.h"

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
#include "uartstdio.h"
#endif

#define VERBOSE

bool GPS_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);

    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    return true;
}

void GPS_Poll()
{
    int32_t c = UARTCharGetNonBlocking(UART1_BASE);
    while (c != -1)
    {
        char s = c & 0xff;
        //UARTwrite(&s, 1);
        c = UARTCharGetNonBlocking(UART1_BASE);
    }
}
