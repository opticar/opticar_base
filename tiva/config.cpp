#include "config.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/eeprom.h"

Configuration g_Cfg;


static const uint32_t CFG_BASE_ADDRESS = 0x100;

#define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]
STATIC_ASSERT(sizeof(Configuration)%4 == 0, config_size_is_multiple_of_4);

bool CFG_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0));

    uint32_t result = EEPROMInit();
    if (result != EEPROM_INIT_OK)
    {
        return false;
    }

    EEPROMRead((uint32_t*)&g_Cfg, CFG_BASE_ADDRESS, sizeof(Configuration));

    return true;
}

void CFG_Save()
{
    EEPROMProgram((uint32_t*)&g_Cfg, CFG_BASE_ADDRESS, sizeof(Configuration));
}
