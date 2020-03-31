#include <Energia.h>

#include <DemoLED.h>

#include <SPI_uDMA_drv.h>
#include <WS2812_drv.h>

DemoLED DemoLed;

#define CHANNEL_RED 1
#define CHANNEL_GREEN 0
#define CHANNEL_BLUE 2

#define LED_OFF 0
#define LED_HALF 127
#define LED_ON 255

DemoLED::DemoLED() {}

void DemoLED::init()
{
  spiDone = 0;

  InitSPITransfer((uint8_t *)spiOut, sizeof(spiOut), &spiDone);

#define LED_COLOR_INIT LED_RED

  setBlockColor(LED_BLOCK_VL, LED_COLOR_INIT);
  setBlockColor(LED_BLOCK_VR, LED_COLOR_INIT);
  setBlockColor(LED_BLOCK_HL, LED_COLOR_INIT);
  setBlockColor(LED_BLOCK_HR, LED_COLOR_INIT);
  setBlockColor(LED_BLOCK_INDL, LED_COLOR_INIT);
  setBlockColor(LED_BLOCK_INDR, LED_COLOR_INIT);
  setBlockColor(LED_BLOCK_REAR, LED_COLOR_INIT);
}

void DemoLED::setLEDsRaw(int start, int count, uint8_t red, uint8_t green,
                         uint8_t blue)
{
  int i = 0;
  start += LED_OFFSET;

  for (i = 0; i < count; ++i)
  {
    showLeds[start + i][CHANNEL_RED] = red;
    showLeds[start + i][CHANNEL_GREEN] = green;
    showLeds[start + i][CHANNEL_BLUE] = blue;

    WSGRBtoSPI(spiOut[start + i], showLeds[start + i][0],
               showLeds[start + i][1], showLeds[start + i][2]);
  }
}

void DemoLED::setLEDsColor(int start, int count, LEDColor color)
{
  uint8_t red = LED_OFF;
  uint8_t green = LED_OFF;
  uint8_t blue = LED_OFF;

  switch (color)
  {
  case LED_RED:
    red = LED_ON;
    break;
  case LED_YELLOW:
    red = LED_ON;
    green = LED_ON;
    break;
  case LED_GREEN:
    green = LED_ON;
    break;
  case LED_CYAN:
    green = LED_ON;
    blue = LED_ON;
    break;
  case LED_BLUE:
    blue = LED_ON;
    break;
  case LED_MAGENTA:
    blue = LED_ON;
    red = LED_ON;
    break;
  case LED_AMBER:
    red = LED_ON;
    green = LED_HALF;
    break;
  case LED_WHITE:
    red = LED_ON;
    green = LED_ON;
    blue = LED_ON;
    break;
  case LED_BLACKOUT:
    break;
  }
  setLEDsRaw(start, count, red, green, blue);
}

void DemoLED::setBlockRaw(LEDBlock block, uint8_t red, uint8_t green,
                          uint8_t blue)
{
  switch (block)
  {
  case LED_BLOCK_VL:
    setLEDsRaw(g_StartFrontLeft, g_CountFrontLeft, red, green, blue);
    break;
  case LED_BLOCK_VR:
    setLEDsRaw(g_StartFrontRight, g_CountFrontRight, red, green, blue);
    break;
  case LED_BLOCK_HL:
    setLEDsRaw(g_StartRearLeft, g_CountRearLeft, red, green, blue);
    break;
  case LED_BLOCK_HR:
    setLEDsRaw(g_StartRearRight, g_CountRearRight, red, green, blue);
    break;
  case LED_BLOCK_INDL:
    setLEDsRaw(g_StartIndiLeft, g_CountIndiLeft, red, green, blue);
    break;
  case LED_BLOCK_INDR:
    setLEDsRaw(g_StartIndiRight, g_CountIndiRight, red, green, blue);
    break;
  case LED_BLOCK_REAR:
    setLEDsRaw(g_StartRear, g_CountRear, red, green, blue);
    break;
  }
}
void DemoLED::setBlockColor(LEDBlock block, LEDColor color)
{
  switch (block)
  {
  case LED_BLOCK_VL:
    setLEDsColor(g_StartFrontLeft, g_CountFrontLeft, color);
    break;
  case LED_BLOCK_VR:
    setLEDsColor(g_StartFrontRight, g_CountFrontRight, color);
    break;
  case LED_BLOCK_HL:
    setLEDsColor(g_StartRearLeft, g_CountRearLeft, color);
    break;
  case LED_BLOCK_HR:
    setLEDsColor(g_StartRearRight, g_CountRearRight, color);
    break;
  case LED_BLOCK_INDL:
    setLEDsColor(g_StartIndiLeft, g_CountIndiLeft, color);
    break;
  case LED_BLOCK_INDR:
    setLEDsColor(g_StartIndiRight, g_CountIndiRight, color);
    break;
  case LED_BLOCK_REAR:
    setLEDsColor(g_StartRear, g_CountRear, color);
    break;
  }
}