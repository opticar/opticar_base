#ifndef SYSTEM_LED_H
#define SYSTEM_LED_H

class SystemLED
{
public:
  SystemLED();
  static void init();
  void on();
  void off();
  void toggle();

private:
  bool currentState;
  int pin;
};

extern SystemLED Led1;
extern SystemLED Led2;
extern SystemLED Led3;
extern SystemLED Led4;
extern SystemLED Led5;
extern SystemLED Led6;
extern SystemLED Led7;

#endif