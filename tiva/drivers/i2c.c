#include "i2c.h"

#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"

bool I2C_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true);
    I2CMasterTimeoutSet(I2C1_BASE, 0xFF);

    SysCtlDelay(10);

    return true;
}

bool I2C_Write(uint8_t addr, uint8_t reg, uint8_t data)
{
    return I2C_WriteBurst(addr, reg, &data, 1);
}

bool I2C_WriteBurst(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t size)
{
    uint8_t i;

    I2CMasterSlaveAddrSet(I2C1_BASE, addr, false);

    I2CMasterDataPut(I2C1_BASE, reg);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C1_BASE))
    {
      if (I2C_Timeout()) return false;
    }

    for (i = 0; i < size - 1; i++)
    {
        I2CMasterDataPut(I2C1_BASE, data[i]);
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while (I2CMasterBusy(I2C1_BASE));
    }

    I2CMasterDataPut(I2C1_BASE, data[size - 1]);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    while (I2CMasterBusy(I2C1_BASE))
    {
      if (I2C_Timeout()) return false;
    }
    
    return true;
}

uint8_t I2C_Read(uint8_t addr, uint8_t reg)
{

    I2CMasterSlaveAddrSet(I2C1_BASE, addr, false);

    I2CMasterDataPut(I2C1_BASE, reg);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C1_BASE))
    {
      if (I2C_Timeout()) return 0xff;
    }

    I2CMasterSlaveAddrSet(I2C1_BASE, addr, true);

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C1_BASE))
    {
      if (I2C_Timeout()) return 0xff;
    }
    return I2CMasterDataGet(I2C1_BASE);
}

bool I2C_ReadBurst(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t size)
{
    uint8_t i;

    I2CMasterSlaveAddrSet(I2C1_BASE, addr, false);

    I2CMasterDataPut(I2C1_BASE, reg);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C1_BASE))
    {
      if (I2C_Timeout()) return false;
    }

    I2CMasterSlaveAddrSet(I2C1_BASE, addr, true);

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (I2CMasterBusy(I2C1_BASE))
    {
      if (I2C_Timeout()) return false;
    }
    data[0] = I2CMasterDataGet(I2C1_BASE);

    for (i = 1; i < size - 1; i++)
    {
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while (I2CMasterBusy(I2C1_BASE))
    {
      if (I2C_Timeout()) return false;
    }
        data[i] = I2CMasterDataGet(I2C1_BASE);
    }

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(I2C1_BASE))
    {
      if (I2C_Timeout()) return false;
    }
    data[size - 1] = I2CMasterDataGet(I2C1_BASE);
    
    return true;
}

bool I2C_Timeout()
{
  if (HWREG(I2C1_BASE + I2C_O_MCS) & I2C_MCS_CLKTO)
  {
    return true;
  }
  else
  {
    return false;
  }
}
