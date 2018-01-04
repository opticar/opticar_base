#include "mpu9250.h"

#include "i2c.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "../config.h"
#ifdef HAVE_UART_PRINTF
#include "uartstdio.h"
#endif

static const uint8_t MPU_REG_RAW_DATA_START = 0x3B;
static const uint8_t MPU_REG_RAW_DATA_ACCEL = 0x3B;
static const uint8_t MPU_REG_RAW_DATA_GYRO = 0x43;
static const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_REG_SMPLRT_DIV = 0x19;
static const uint8_t MPU_REG_CONFIG = 0x1A;
static const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
static const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_REG_ACCEL_CONFIG2 = 0x1D;
static const uint8_t MPU_REG_INT_PIN_CFG = 0x37;
static const uint8_t MPU_REG_INT_ENABLE = 0x38;

static const uint8_t MAG_I2C_ADDR = 0x0C;
static const uint8_t MAG_REG_STATUS1 = 0x02;
static const uint8_t MAG_REG_RAW_DATA_START = 0x03;
static const uint8_t MAG_REG_STATUS2 = 0x09;
static const uint8_t MAG_REG_CONTROL1 = 0x0A;
static const uint8_t MAG_REG_CONTROL2 = 0x0B;

static SensorDataRaw g_GyroCalib;

static bool IsDataVariationInRange(int32_t* data, uint32_t length, int32_t maxDifference)
{
    if (!data) return false;

    int32_t min = data[0];
    int32_t max = data[0];

    for (uint32_t i = 1; i < length; ++i)
    {
        if (data[i] < min) min = data[i];
        else if (data[i] > max) max = data[i];
    }

    return ((max - min) < maxDifference);
}

static bool CalibrateSensor(SensorDataRaw* zeroData, uint8_t baseRegister, int32_t maxDifference)
{
    static const uint32_t numValues = 32;
    int32_t dataBuffer[3][numValues];

    uint8_t rawBuffer[6];

    for (uint32_t i = 0 ; i< numValues; ++i)
    {
        while (!MPU_DataReady());

        I2C_ReadBurst(MPU_I2C_ADDRESS, baseRegister, rawBuffer, 6);
        dataBuffer[0][i] = (int16_t)((rawBuffer[0] << 8) | rawBuffer[1]);
        dataBuffer[1][i] = (int16_t)((rawBuffer[2] << 8) | rawBuffer[3]);
        dataBuffer[2][i] = (int16_t)((rawBuffer[4] << 8) | rawBuffer[5]);
        SysCtlDelay(SysCtlClockGet() / 300); // Wait for 10ms
    }

    for (int axis = 0; axis < 3; ++axis)
    {
        if (!IsDataVariationInRange(dataBuffer[axis], numValues, maxDifference)) return false;
    }

    for (int axis = 0; axis < 3; ++axis)
    {
        for (uint32_t i = 1; i < numValues; ++i)
        {
            dataBuffer[axis][0] += dataBuffer[axis][i];
        }

        zeroData->m_Data[axis] = dataBuffer[axis][0] / numValues;
    }

    return true;
}

static bool CalibrateGyro()
{
    bool result = CalibrateSensor(&g_GyroCalib, MPU_REG_RAW_DATA_GYRO, 100); // 100 raw value equals approx. 6.1 deg/s

    if (result)
    {
#ifdef HAVE_UART_PRINTF
        UARTprintf("Gyro zero values: %d X, %d Y, %d Z\n", g_GyroCalib.m_Axis.m_X, g_GyroCalib.m_Axis.m_Y, g_GyroCalib.m_Axis.m_Z);
#endif
    }
    else
    {
#ifdef HAVE_UART_PRINTF
        UARTprintf("Gyro calibration failed\n");
#endif
    }

    return result;
}

static bool CalibrateAccel(MPU9250* mpu)
{
    bool result = CalibrateSensor((SensorDataRaw*)(&g_Cfg.m_AcceleratorCalibX), MPU_REG_RAW_DATA_ACCEL, 100); // 100 raw value equals approx. 0.02g
    if (result)
    {
        g_Cfg.m_AcceleratorCalibZ -= mpu->m_AccelScale;
#ifdef HAVE_UART_PRINTF
        UARTprintf("Accel zero values: %d X, %d Y, %d Z\n", g_Cfg.m_AcceleratorCalibX, g_Cfg.m_AcceleratorCalibY, g_Cfg.m_AcceleratorCalibZ);
#endif
    }
    else
    {
#ifdef HAVE_UART_PRINTF
        UARTprintf("Accel calibration failed\n");
#endif
    }

    return result;
}

static bool IsMagnetometerDataReady()
{
  uint8_t reg = I2C_Read(MAG_I2C_ADDR, MAG_REG_STATUS1);
  if (reg == 0xff) return false; // I2C failure

  return (reg & 0x01);
}

bool MPU_Init(MPU9250* mpu)
{
    if (!mpu) return false;

    uint8_t id = I2C_Read(MPU_I2C_ADDRESS, MPU_REG_WHOAMI);
    if (id != MPU_VAL_WHOAMI)
    {
        return false;
    }

    // Reset MPU
    I2C_Write(MPU_I2C_ADDRESS, MPU_REG_PWR_MGMT_1, 0x80);
    SysCtlDelay(SysCtlClockGet() / 30); // Wait for 100ms

    while (I2C_Read(MPU_I2C_ADDRESS, MPU_REG_PWR_MGMT_1) & 0x80);

    // Enable PLL
    I2C_Write(MPU_I2C_ADDRESS, MPU_REG_PWR_MGMT_1, 0x01);

    // Configure MPU
    I2C_Write(MPU_I2C_ADDRESS, MPU_REG_SMPLRT_DIV, 0x0); // 1kHz sampling rate
    I2C_Write(MPU_I2C_ADDRESS, MPU_REG_CONFIG, 0x3); // Select 41 Hz Gyro, 1kHz sampling, 42Hz Temp
    I2C_Write(MPU_I2C_ADDRESS, MPU_REG_GYRO_CONFIG, 0x18); // Select 2000dps full scale
    I2C_Write(MPU_I2C_ADDRESS, MPU_REG_ACCEL_CONFIG, 0x10); // Select 8g full scale
    I2C_Write(MPU_I2C_ADDRESS, MPU_REG_ACCEL_CONFIG2, 0x3); // Select 41 Hz Accel, 1kHz sampling

    mpu->m_GyroScale = 16.4f; // Scale factor for 2000deg/s
    mpu->m_AccelScale = 4096.0f; // Scale factor for 8g
    mpu->m_MagScale = (32760.0f / 4912.0f) / 1000000.0f; // Scale factor for 4912 Microtesla at full value (32760) and conversion to Tesla

    I2C_Write(MPU_I2C_ADDRESS, MPU_REG_INT_PIN_CFG, 0x32); // Set interrupt state to level, keep high until read; select I2C passthrough mode for AK8963 access
    I2C_Write(MPU_I2C_ADDRESS, MPU_REG_INT_ENABLE, 0x1);

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);

    SysCtlDelay(SysCtlClockGet() / 30); // Wait for 100ms

    while (!CalibrateGyro());

    int tries = 10;
    bool result = false;

    do
    {
        result = CalibrateAccel(mpu);
        --tries;
    } while (!result && (tries >= 0));

    // Initialize AK8963
    mpu->m_MagWhoAmI = I2C_Read(MAG_I2C_ADDR, 0x00);
    I2C_Write(MAG_I2C_ADDR, MAG_REG_CONTROL2, 0x01);
    uint8_t magStatus2 = 0;
    do
    {
      magStatus2 = I2C_Read(MAG_I2C_ADDR, MAG_REG_CONTROL2);
    } while((magStatus2 != 0xff) && (magStatus2 & 0x01))
    ; // Wait for soft reset
    I2C_Write(MAG_I2C_ADDR, MAG_REG_CONTROL1, 0x16); // Set to 100Hz continuous reading and 16bit output mode
    
    CFG_Save();

    return true;
}

bool MPU_DataReady()
{
    return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5);
}

void MPU_GetData(MPU9250* mpu)
{
    if (!mpu) return;

    uint8_t rawData[14];

    // Read accelerometer and gyrometer
    I2C_ReadBurst(MPU_I2C_ADDRESS, MPU_REG_RAW_DATA_START, rawData, 14);

    mpu->m_Accel.m_Axis.m_X = (int16_t)((rawData[0] << 8) | rawData[1]) - g_Cfg.m_AcceleratorCalibX;
    mpu->m_Accel.m_Axis.m_Y = (int16_t)((rawData[2] << 8) | rawData[3]) - g_Cfg.m_AcceleratorCalibY;
    mpu->m_Accel.m_Axis.m_Z = (int16_t)((rawData[4] << 8) | rawData[5]) - g_Cfg.m_AcceleratorCalibZ;

    static const float g = -9.81f;

    mpu->m_AccelInEgoFrame.m_Axis.m_X = mpu->m_Accel.m_Axis.m_X / mpu->m_AccelScale * g;
    mpu->m_AccelInEgoFrame.m_Axis.m_Y = mpu->m_Accel.m_Axis.m_Y / mpu->m_AccelScale * g;
    mpu->m_AccelInEgoFrame.m_Axis.m_Z = mpu->m_Accel.m_Axis.m_Z / mpu->m_AccelScale * g;

    mpu->m_Gyro.m_Axis.m_X = (int16_t)((rawData[8] << 8)  | rawData[9])  - g_GyroCalib.m_Axis.m_X;
    mpu->m_Gyro.m_Axis.m_Y = (int16_t)((rawData[10] << 8) | rawData[11]) - g_GyroCalib.m_Axis.m_Y;
    mpu->m_Gyro.m_Axis.m_Z = (int16_t)((rawData[12] << 8) | rawData[13]) - g_GyroCalib.m_Axis.m_Z;

    mpu->m_GyroRates.m_Angles.m_AX = (float)mpu->m_Gyro.m_Axis.m_X / mpu->m_GyroScale;
    mpu->m_GyroRates.m_Angles.m_AY = (float)mpu->m_Gyro.m_Axis.m_Y / mpu->m_GyroScale;
    mpu->m_GyroRates.m_Angles.m_AZ = (float)mpu->m_Gyro.m_Axis.m_Z / mpu->m_GyroScale;

    // Read magnetometer
    for (int i = 0; i < 7; ++i) rawData[i] = 0;

    if (IsMagnetometerDataReady())
    {
      I2C_ReadBurst(MAG_I2C_ADDR, MAG_REG_RAW_DATA_START, rawData, 7); // Read past data registers to signal end of transmission to device

      mpu->m_Mag.m_Axis.m_X = (int16_t)((rawData[1] << 8) | rawData[0]) - g_Cfg.m_MagnetCalibX;
      mpu->m_Mag.m_Axis.m_Y = (int16_t)((rawData[3] << 8) | rawData[2]) - g_Cfg.m_MagnetCalibY;
      mpu->m_Mag.m_Axis.m_Z = (int16_t)((rawData[5] << 8) | rawData[4]) - g_Cfg.m_MagnetCalibZ;

      // Convert to same coordinate frame as accelerometer
      mpu->m_MagInEgoFrame.m_Axis.m_X =  mpu->m_Mag.m_Axis.m_Y;// / mpu->m_MagScale;
      mpu->m_MagInEgoFrame.m_Axis.m_Y =  mpu->m_Mag.m_Axis.m_X;// / mpu->m_MagScale;
      mpu->m_MagInEgoFrame.m_Axis.m_Z = -mpu->m_Mag.m_Axis.m_Z;// / mpu->m_MagScale;
    }
    else
    {
      mpu->m_MagInEgoFrame.m_Axis.m_X = mpu->m_MagScale;
      mpu->m_MagInEgoFrame.m_Axis.m_Y = mpu->m_MagScale;
      mpu->m_MagInEgoFrame.m_Axis.m_Z = mpu->m_MagScale;
    }
}
