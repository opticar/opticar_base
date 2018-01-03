#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LED_OFFSET 0
#define NUM_SHOW_LEDS 41 + LED_OFFSET // 6 VR, 2 BlinkR, 5 HR, 15 H, 5 HL, 2 BlinkL, 6 VL = 41

#define g_CountFrontLeft  6
#define g_CountFrontRight 6
#define g_CountRearLeft   5
#define g_CountRearRight  5
#define g_CountIndiLeft   2
#define g_CountIndiRight  2
#define g_CountRear      13

#define g_StartFrontRight (0)
#define g_StartIndiRight  (g_StartFrontRight + g_CountFrontRight)
#define g_StartRearRight  (g_StartIndiRight + g_CountIndiRight)
#define g_StartRear       (g_StartRearRight + g_CountRearRight)
#define g_StartRearLeft   (g_StartRear + g_CountRear)
#define g_StartIndiLeft   (g_StartRearLeft + g_CountRearLeft)
#define g_StartFrontLeft  (g_StartIndiLeft + g_CountIndiLeft)

typedef enum
{
    LED_BLOCK_VL,
    LED_BLOCK_VR,
    LED_BLOCK_HL,
    LED_BLOCK_HR,
    LED_BLOCK_INDL,
    LED_BLOCK_INDR,
    LED_BLOCK_REAR
} LEDBlock;

typedef enum
{
    LED_BLACKOUT = 0,
    LED_RED,
    LED_YELLOW,
    LED_GREEN,
    LED_CYAN,
    LED_BLUE,
    LED_MAGENTA,
    LED_AMBER,
    LED_WHITE
} LEDColor;

typedef enum
{
    SYSTEM_LED_1,
    SYSTEM_LED_2,
    SYSTEM_LED_3,
    SYSTEM_LED_4,
    SYSTEM_LED_HEARTBEAT,
    SYSTEM_LED_EMERGENCYSTOP,
    SYSTEM_LED_READY
} SystemLED;

bool LED_Init();
void LED_SetLEDBlock(int start, int count, uint8_t red, uint8_t green, uint8_t blue);
void LED_SetLEDBlockColor(int start, int count, LEDColor color);
void LED_Set(LEDBlock block, LEDColor color);

void LED_SetSystemLED(SystemLED led, bool on);

#ifdef __cplusplus
}
#endif
