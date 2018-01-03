#include "CAN.hpp"

#include "inc/hw_can.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"

#include "uartstdio.h"

#include "motors.h"

//#define VERBOSE

typedef struct
{
    int16_t m_VL_raw;
    int16_t m_VR_raw;
    int16_t m_HL_raw;
    int16_t m_HR_raw;
} PayloadMotorDirect;

extern uint32_t g_SystemTickCounter;
extern const int32_t SYS_TICK_HZ;

extern "C"
{

CAN* g_CAN;

void CAN_HandleInterrupt()
{
    g_CAN->HandleInterrupt();
}
} // extern C

CAN::CAN()
{
}

CAN::~CAN()
{
}

bool CAN::Initialize()
{
    m_LastErrors = 0;
    m_LastStatus = 0;
    m_RxFlag = false;

    m_TimeoutAtSysTicks = 0;

    GPIOPinConfigure(GPIO_PE4_CAN0RX);
    GPIOPinConfigure(GPIO_PE5_CAN0TX);

    GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);// Enable the GPIO pin for the PB4.  Set the direction as output with LOW
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x0);// This is necessary for CAN Transceiver MCP2561 (PIN8 of MCP2561 is standby signal for CAN)

    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    CANInit(CAN0_BASE);

    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);

    CANIntRegister(CAN0_BASE, CAN_HandleInterrupt);

    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    IntEnable(INT_CAN0);

    CANEnable(CAN0_BASE);

    m_RxMsg.ui32MsgID = 0x10000;
    m_RxMsg.ui32MsgIDMask = 0x1ffff;
    m_RxMsg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_USE_EXT_FILTER;
    m_RxMsg.ui32MsgLen = 8;
    m_RxMsg.pui8MsgData = m_RxData;

    CANMessageSet(CAN0_BASE, 1, &m_RxMsg, MSG_OBJ_TYPE_RX);

    return true;
}

void CAN::Shutdown()
{
    CANDisable(CAN0_BASE);
}

void CAN::Iterate()
{
    if (m_RxFlag)
    {
        m_RxMsg.ui32MsgLen = 8;
        m_RxMsg.pui8MsgData = m_RxData;

        CANMessageGet(CAN0_BASE, 1, &m_RxMsg, 0);

        m_RxFlag = false;

        if (m_RxMsg.ui32Flags & MSG_OBJ_DATA_LOST)
        {
            UARTprintf("\n CAN message data loss\n");
        }

#ifdef VERBOSE
        int i = 0;

        UARTprintf("Got can message 0x%0x with DLC %d", m_RxMsg.ui32MsgID, m_RxMsg.ui32MsgLen);
        if (m_RxMsg.ui32MsgLen > 0)
        {
            if (m_RxMsg.ui32MsgLen > 8)
            {
                m_RxMsg.ui32MsgLen = 8;
            }
            UARTprintf(" and data 0x");
            for (i = 0; i < m_RxMsg.ui32MsgLen; ++i)
            {
                UARTprintf("%02X", m_RxData[i]);
            }
        }
        UARTprintf("\n");
#endif

        switch (m_RxMsg.ui32MsgID)
        {
        case 0x10000:
            HandleMotorDirect();
            break;
        default:
            break;
        }
    }
    else
    {
        if (m_LastErrors != 0)
        {
            HandleErrors();
        }
    }

    if (m_TimeoutAtSysTicks <= g_SystemTickCounter)
    {
        MOTOR_EmergencyStop();
    }
}

void CAN::HandleInterrupt()
{
    uint32_t cause = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    if (cause == CAN_INT_INTID_STATUS)
    {
        m_LastStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        m_LastErrors |= m_LastStatus;
    }
    else if (cause >= 0x01 && cause <= 0x20)
    {
        CANIntClear(CAN0_BASE, cause);

        m_RxFlag = true;

        m_LastErrors = 0;

    }
    else
    {
        UARTprintf("Spurious CAN interrupt 0x%X\n", cause);
    }
}

void CAN::HandleErrors()
{
    if (m_LastErrors & CAN_STATUS_BUS_OFF)
    {
        UARTprintf("ERROR: CAN_STATUS_BUS_OFF\n");

        m_LastErrors &= ~CAN_STATUS_BUS_OFF;
    }

    if (m_LastErrors & CAN_STATUS_EWARN)
    {
        UARTprintf("ERROR: CAN_STATUS_EWARN\n");

        m_LastErrors &= ~CAN_STATUS_EWARN;
    }

    if (m_LastErrors & CAN_STATUS_EPASS)
    {
        UARTprintf("ERROR: CAN_STATUS_EPASS\n");

        m_LastErrors &= ~CAN_STATUS_EPASS;
    }

    if (m_LastErrors & CAN_STATUS_RXOK)
    {
        // Reception ok

        m_LastErrors &= ~CAN_STATUS_RXOK;
    }

    if (m_LastErrors & CAN_STATUS_TXOK)
    {
        // Transmission ok

        m_LastErrors &= ~CAN_STATUS_TXOK;
    }

    if (m_LastErrors & CAN_STATUS_LEC_MSK)
    {
        UARTprintf("ERROR: CAN_STATUS_LEC_MSK\n");

        m_LastErrors &= ~CAN_STATUS_LEC_MSK;
    }

    if (m_LastErrors & CAN_STATUS_LEC_STUFF)
    {
        UARTprintf("ERROR: CAN_STATUS_LEC_STUFF\n");

        m_LastErrors &= ~CAN_STATUS_LEC_STUFF;
    }

    if (m_LastErrors & CAN_STATUS_LEC_FORM)
    {
        UARTprintf("ERROR: CAN_STATUS_LEC_FORM\n");

        m_LastErrors &= ~CAN_STATUS_LEC_FORM;
    }

    if (m_LastErrors & CAN_STATUS_LEC_ACK)
    {
        UARTprintf("ERROR: CAN_STATUS_LEC_ACK\n");

        m_LastErrors &= ~CAN_STATUS_LEC_ACK;
    }

    if (m_LastErrors & CAN_STATUS_LEC_BIT1)
    {
        UARTprintf("ERROR: CAN_STATUS_LEC_BIT1\n");

        m_LastErrors &= ~CAN_STATUS_LEC_BIT1;
    }

    if (m_LastErrors & CAN_STATUS_LEC_BIT0)
    {
        UARTprintf("ERROR: CAN_STATUS_LEC_BIT0\n");

        m_LastErrors &= ~CAN_STATUS_LEC_BIT0;
    }

    if (m_LastErrors & CAN_STATUS_LEC_CRC)
    {
        UARTprintf("ERROR: CAN_STATUS_LEC_CRC\n");

        m_LastErrors &= ~CAN_STATUS_LEC_CRC;
    }

    if (m_LastErrors)
    {
        UARTprintf("ERROR: Unhandled error 0x%X\n", m_LastErrors);
    }
}

void CAN::HandleMotorDirect()
{
    MOTOR_ReleaseEmergencyStop();

    PayloadMotorDirect* pl = (PayloadMotorDirect*)m_RxData;

    int front_leftPerc = 0;
    int front_rightPerc = 0;
    int rear_leftPerc = 0;
    int rear_rightPerc = 0;

    front_leftPerc = (pl->m_VL_raw)*0.048828;
    front_rightPerc = (pl->m_VR_raw)*0.048828;
    rear_leftPerc = (pl->m_HL_raw)*0.048828;
    rear_rightPerc = (pl->m_HR_raw)*0.048828;
#ifdef VERBOSE
    // write to UART for debugging
    UARTprintf("Received motor power in percent:\n front_leftPerc: %d\n front_rightPerc: %d\n rear_leftPerc: %d\n rear_rightPerc: %d\n", front_leftPerc, front_rightPerc, rear_leftPerc, rear_rightPerc);
#endif

    // set motor
    MOTOR_SetSpeed(FRONT_LEFT, front_leftPerc);
    MOTOR_SetSpeed(FRONT_RIGHT,front_rightPerc);
    MOTOR_SetSpeed(REAR_LEFT, rear_leftPerc);
    MOTOR_SetSpeed(REAR_RIGHT, rear_rightPerc);

    m_TimeoutAtSysTicks = g_SystemTickCounter + (SYS_TICK_HZ / 5); // Timeout is 200ms
}
