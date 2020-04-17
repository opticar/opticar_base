#include "Encoder.h"

static SingleEncoder* encoderToPins[64] = { nullptr };
#define GENERATE_ENCODER_IH(n)                                                                                         \
  void encoderInterrupt##n()                                                                                           \
  {                                                                                                                    \
    encoderToPins[n]->interruptHandler();                                                                              \
  }

#define GENERATE_ATTACH(n)                                                                                             \
  case n:                                                                                                              \
    attachInterrupt(digitalPinToInterrupt(pin1), encoderInterrupt##n, RISING);                                         \
    break;

GENERATE_ENCODER_IH(1)
GENERATE_ENCODER_IH(2)
GENERATE_ENCODER_IH(3)
GENERATE_ENCODER_IH(4)
GENERATE_ENCODER_IH(5)
GENERATE_ENCODER_IH(6)
GENERATE_ENCODER_IH(7)
GENERATE_ENCODER_IH(8)
GENERATE_ENCODER_IH(9)
GENERATE_ENCODER_IH(10)
GENERATE_ENCODER_IH(11)
GENERATE_ENCODER_IH(12)
GENERATE_ENCODER_IH(13)
GENERATE_ENCODER_IH(14)
GENERATE_ENCODER_IH(15)
GENERATE_ENCODER_IH(16)
GENERATE_ENCODER_IH(17)
GENERATE_ENCODER_IH(18)
GENERATE_ENCODER_IH(19)
GENERATE_ENCODER_IH(20)
GENERATE_ENCODER_IH(21)
GENERATE_ENCODER_IH(22)
GENERATE_ENCODER_IH(23)
GENERATE_ENCODER_IH(24)
GENERATE_ENCODER_IH(25)
GENERATE_ENCODER_IH(26)
GENERATE_ENCODER_IH(27)
GENERATE_ENCODER_IH(28)
GENERATE_ENCODER_IH(29)
GENERATE_ENCODER_IH(30)
GENERATE_ENCODER_IH(31)
GENERATE_ENCODER_IH(32)
GENERATE_ENCODER_IH(33)
GENERATE_ENCODER_IH(34)
GENERATE_ENCODER_IH(35)
GENERATE_ENCODER_IH(36)
GENERATE_ENCODER_IH(37)
GENERATE_ENCODER_IH(38)
GENERATE_ENCODER_IH(39)
GENERATE_ENCODER_IH(40)

SingleEncoder::SingleEncoder(uint32_t pin1, int countsPerRev)
  : counter(0), prevCounter(0), countsPerRev(countsPerRev), prevUpdateTime(0)
{
  encoderToPins[pin1] = this;
  pinMode(pin1, INPUT_PULLUP);
  switch (pin1)
  {
    GENERATE_ATTACH(1)
    GENERATE_ATTACH(2)
    GENERATE_ATTACH(3)
    GENERATE_ATTACH(4)
    GENERATE_ATTACH(5)
    GENERATE_ATTACH(6)
    GENERATE_ATTACH(7)
    GENERATE_ATTACH(8)
    GENERATE_ATTACH(9)
    GENERATE_ATTACH(10)
    GENERATE_ATTACH(11)
    GENERATE_ATTACH(12)
    GENERATE_ATTACH(13)
    GENERATE_ATTACH(14)
    GENERATE_ATTACH(15)
    GENERATE_ATTACH(16)
    GENERATE_ATTACH(17)
    GENERATE_ATTACH(18)
    GENERATE_ATTACH(19)
    GENERATE_ATTACH(20)
    GENERATE_ATTACH(21)
    GENERATE_ATTACH(22)
    GENERATE_ATTACH(23)
    GENERATE_ATTACH(24)
    GENERATE_ATTACH(25)
    GENERATE_ATTACH(26)
    GENERATE_ATTACH(27)
    GENERATE_ATTACH(28)
    GENERATE_ATTACH(29)
    GENERATE_ATTACH(30)
    GENERATE_ATTACH(31)
    GENERATE_ATTACH(32)
    GENERATE_ATTACH(33)
    GENERATE_ATTACH(34)
    GENERATE_ATTACH(35)
    GENERATE_ATTACH(36)
    GENERATE_ATTACH(37)
    GENERATE_ATTACH(38)
    GENERATE_ATTACH(39)
    GENERATE_ATTACH(40)

    default:
      break;
  }
}

int SingleEncoder::getRPM()
{
  uint32_t currentCounter = read();
  unsigned long currentTime = millis();
  unsigned long dT = currentTime - prevUpdateTime;  // Will only overflow after approx. 49 days

  double dTminutes = (double)dT / (60 * 1000);
  double deltaTicks = currentCounter - prevCounter;
  // If this overflows, output a zero speed once
  if (deltaTicks < 0)
    deltaTicks = 0;

  prevUpdateTime = currentTime;
  prevCounter = counter;

  return (deltaTicks / countsPerRev) / dTminutes;
}

uint32_t SingleEncoder::read()
{
  return counter;
}

void SingleEncoder::interruptHandler()
{
  ++counter;
}
