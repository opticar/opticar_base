#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    int16_t m_AcceleratorCalibX;
    int16_t m_AcceleratorCalibY;
    int16_t m_AcceleratorCalibZ;

    uint16_t m_Dummy;

    float m_MagnetCalibX;
    float m_MagnetCalibY;
    float m_MagnetCalibZ;
} __attribute__((packed)) Configuration;

extern Configuration g_Cfg;

bool CFG_Init();
void CFG_Save();

#ifdef __cplusplus
}
#endif
