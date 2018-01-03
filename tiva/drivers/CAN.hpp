#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/can.h"

class CAN
{
public:
    bool Initialize();
    void Shutdown();
    void Iterate();
    void HandleInterrupt();

    CAN();
    ~CAN();

protected:
    void HandleErrors();
    void HandleMotorDirect();

    bool m_Initialized();

    CAN(const CAN& rhs);
    CAN& operator=(const CAN& rhs);

    uint32_t m_LastErrors;
    uint32_t m_LastStatus;
    bool m_RxFlag;
    tCANMsgObject m_RxMsg;
    uint8_t m_RxData[8];

    int32_t m_TimeoutAtSysTicks;
};
