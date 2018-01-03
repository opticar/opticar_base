#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef union
{
    struct
    {
        int16_t m_X;
        int16_t m_Y;
        int16_t m_Z;
    } __attribute__((packed)) m_Axis;
    int16_t m_Data[3];
} SensorDataRaw;

typedef union
{
    struct
    {
        float m_X;
        float m_Y;
        float m_Z;
    } __attribute__((packed)) m_Axis;
    float m_Data[3];
} SensorDataAxis;

typedef union
{
    struct
    {
        float m_AX;
        float m_AY;
        float m_AZ;
    } __attribute__((packed)) m_Angles;
    int16_t m_Data[3];
} SensorDataAngles;

typedef struct
{
    SensorDataRaw m_Accel;
    SensorDataRaw m_Gyro;
    SensorDataRaw m_Mag;

    SensorDataAxis m_AccelInEgoFrame;
    SensorDataAngles m_GyroRates;
    SensorDataAxis m_MagInEgoFrame;

    float m_AccelScale;
    float m_GyroScale;
    float m_MagScale;
  
    uint8_t m_MagWhoAmI;
} MPU9250;

static const uint8_t MPU_I2C_ADDRESS = 0x68;
static const uint8_t MPU_REG_WHOAMI = 0x75;
static const uint8_t MPU_VAL_WHOAMI = 0x71;

bool MPU_Init(MPU9250* mpu);
bool MPU_DataReady();
void MPU_GetData(MPU9250* mpu);

#ifdef __cplusplus
}
#endif

