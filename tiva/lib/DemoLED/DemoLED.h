#ifndef DEMO_LED_H
#define DEMO_LED_H

#include OPTICAR_CFG

#define LED_OFFSET 0
#define NUM_SHOW_LEDS                                                                                                  \
  LED_OFFSET + g_CountFrontLeft + g_CountFrontRight + g_CountRearLeft + g_CountRearRight + g_CountIndiLeft +           \
      g_CountIndiRight + g_CountRear

#define g_StartFrontRight (0)
#define g_StartIndiRight (g_StartFrontRight + g_CountFrontRight)
#define g_StartRearRight (g_StartIndiRight + g_CountIndiRight)
#define g_StartRear (g_StartRearRight + g_CountRearRight)
#define g_StartRearLeft (g_StartRear + g_CountRear)
#define g_StartIndiLeft (g_StartRearLeft + g_CountRearLeft)
#define g_StartFrontLeft (g_StartIndiLeft + g_CountIndiLeft)

/// A class to access the demonstrator's LED stripes
class DemoLED
{
public:
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

public:
  /// Empty constructor
  DemoLED();
  /// Initialize the demonstrator LED subsystem
  void init();

  void setLEDsRaw(int start, int count, uint8_t red, uint8_t green, uint8_t blue);
  void setLEDsColor(int start, int count, LEDColor color);
  void setBlockRaw(LEDBlock block, uint8_t red, uint8_t green, uint8_t blue);
  void setBlockColor(LEDBlock block, LEDColor color);

private:
  uint8_t showLeds[NUM_SHOW_LEDS][3];
  uint8_t spiOut[NUM_SHOW_LEDS][3 * 8];  // 3 colors, 8 bits per color
  uint8_t spiDone;
};

// Demonstrator LED stripe segments
extern DemoLED DemoLed;

#endif