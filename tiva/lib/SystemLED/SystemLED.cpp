#include <Energia.h>

#include "SystemLED.h"

// Map system LEDs to pin numbers (see pins_energia.h)
#define SYSTEM_LED_1_PIN 17
#define SYSTEM_LED_2_PIN NOT_A_PIN  // Not available on revision V2
#define SYSTEM_LED_3_PIN 15
#define SYSTEM_LED_4_PIN 14
#define SYSTEM_LED_5_PIN 13
#define SYSTEM_LED_6_PIN 12
#define SYSTEM_LED_7_PIN 11

SystemLED Led1;
SystemLED Led2;
SystemLED Led3;
SystemLED Led4;
SystemLED LedHeartbeat;
SystemLED LedEmergencyStop;
SystemLED LedReady;

SystemLED::SystemLED() : currentState(false), pin(NOT_A_PIN)
{
}

void SystemLED::init()
{
  // Set pin directions and initialize to off
  pinMode(SYSTEM_LED_1_PIN, OUTPUT);
  pinMode(SYSTEM_LED_2_PIN, OUTPUT);
  pinMode(SYSTEM_LED_3_PIN, OUTPUT);
  pinMode(SYSTEM_LED_4_PIN, OUTPUT);
  pinMode(SYSTEM_LED_5_PIN, OUTPUT);
  pinMode(SYSTEM_LED_6_PIN, OUTPUT);
  pinMode(SYSTEM_LED_7_PIN, OUTPUT);

  Led1.pin = SYSTEM_LED_1_PIN;
  Led2.pin = SYSTEM_LED_2_PIN;
  Led3.pin = SYSTEM_LED_3_PIN;
  Led4.pin = SYSTEM_LED_4_PIN;
  LedHeartbeat.pin = SYSTEM_LED_5_PIN;
  LedEmergencyStop.pin = SYSTEM_LED_6_PIN;
  LedReady.pin = SYSTEM_LED_7_PIN;

  Led1.off();
  Led2.off();
  Led3.off();
  Led4.off();
  LedHeartbeat.off();
  LedEmergencyStop.off();
  LedReady.off();
}

void SystemLED::on()
{
  currentState = true;
  digitalWrite(pin, currentState);
}

void SystemLED::off()
{
  currentState = false;
  digitalWrite(pin, currentState);
}

void SystemLED::toggle()
{
  currentState = !currentState;
  digitalWrite(pin, currentState);
}