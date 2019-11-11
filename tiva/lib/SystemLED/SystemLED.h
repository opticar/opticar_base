#ifndef SYSTEM_LED_H
#define SYSTEM_LED_H

/// A class to access the onboard LEDs on the Tiva carrier board
class SystemLED
{
public:
  /// Empty constructor, call SystemLED::init() instead
  SystemLED();
  /// Initialize the predefined objects
  static void init();
  /// Switch the LED on
  void on();
  /// Switch the LED off
  void off();
  /// Toggle the LED
  void toggle();

private:
  /// Energia/Arduino pin number for this LED
  int pin;
};

// First LED, closest to the 3-pole pin headers
extern SystemLED Led1;
// Not accessible in revision V2
extern SystemLED Led2;
// Third LED
extern SystemLED Led3;
// Fourth LED
extern SystemLED Led4;
// Fifth LED, default use is a hearbeat signal
extern SystemLED LedHeartbeat;
// Sixth LED, default use is indicating an emergency stop
extern SystemLED LedEmergencyStop;
// Seventh LED, default use is indicating an existing connection
extern SystemLED LedReady;

#endif