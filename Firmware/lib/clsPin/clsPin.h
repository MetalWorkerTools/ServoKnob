#pragma once

#include <Arduino.h>
#include <driver/ledc.h>
#include <esp_types.h>
// #include <stdatomic.h>

#ifndef DEBOUNCETIME
#define DEBOUNCETIME 100
#endif
#ifndef SHORT_HOLD_MILLI_SECONDS
#define SHORT_HOLD_MILLI_SECONDS 0
#endif
#ifndef LONG_HOLD_MILLI_SECONDS
#define LONG_HOLD_MILLI_SECONDS 0
#endif
enum PinStates
{
  PinStateU,  // Undefined
  PinStateUA, // Undefined, changing to Active
  PinStateUI, // Undefined, changing to Inactive

  PinStateA,   // Active, coming from processed state
  PinStateAI,  // Active, changing to Inactive
  PinStateAU,  // Active, coming from unprocessed active state
  PinStateAUI, // Active, coming from unprocessed inactive staten, changing to Inactive
  PinStateAP,  // Active and processed
  PinStateAPI, // Active and processed, changing to Inactive

  PinStateI,   // Inactive, coming from processed state
  PinStateIA,  // Inactive, changing to Active
  PinStateIU,  // Inactive, coming from unprocessed active
  PinStateIUA, // Inactive, coming from unprocessed active, changing to Active
  PinStateIP,  // Inactive and processed
  PinStateIPA, // Inactive and processed, changing to Active

  PinStateS,   // Short time active
  PinStateSI,  // Short time active, changing to Inactive
  PinStateSP,  // Short time active and processed
  PinStateSPI, // Short time active and processed, changing to Inactive

  PinStateIS,  // Inactive, coming from unprocessed short time active
  PinStateISA, // Inactive coming from unprocessed short time active, changing to Active
};
class clsPin
{
public:
  PinStates PinState = PinStateU; // At start the pinstate is undefined
  uint8_t Pin = 0;
  uint8_t Mode = INPUT;
  uint8_t ActiveLevel = 0;
  uint8_t Level = 0;
  uint8_t State = 0;
  uint32_t ResolutionBits = 10;
  uint32_t PeriodTics = 1024;
  uint32_t Frequency = 1;
  uint32_t DutyCycle = 10;
  uint32_t PeriodUs = 100000;

  String strLow = "L";
  String strHigh = "H";
  String strActive = "A";
  String strInactive = "a";

  ledc_timer_config_t ledc_timer;
  ledc_channel_config_t ledc_channel;

  unsigned long ShortHoldMilliseconds = SHORT_HOLD_MILLI_SECONDS; // how long a short press shoud be. Do not set too low to avoid bouncing (false press events).
  unsigned long longHoldMiliseconds = LONG_HOLD_MILLI_SECONDS;    // how long a long press shoud be. Do not set too low to avoid not registering short presses
  volatile unsigned long lastTimeActive = millis();
  volatile unsigned long lastTimeChange = millis();
  volatile unsigned long PWMlastTimeInActive = micros();
  volatile unsigned long PWMlastTimeActive = micros();
  volatile unsigned long PWMperiodTime = 1;
  volatile unsigned long PWMpulseTime = 1;
  volatile bool PulseProcessed = false;
  boolean PulseSignalActive();
  long GetPWMfrequency();
  long GetPWMdutyCycle();
  long GetPWMpulseTime();
  long GetPWMperiodTime();

  volatile bool ProcessShortHoldActive = false;
  volatile bool ProcessLongHoldActive = false;
  unsigned long DebounceTime = 5; // Default 5 msec debounce time
  volatile bool PinStable = false;
  clsPin(uint8_t Pin, uint8_t Mode, uint8_t ActiveLevel, uint8_t PinState, String strActive, String strInactive, String strLow, String strHigh, unsigned long ShortPressMilliseconds);
  clsPin(uint8_t Pin, uint8_t Mode, uint8_t ActiveLevel, uint8_t PinState, String strActive, String strInactive, unsigned long ShortHoldMilliseconds) : clsPin(Pin, Mode, ActiveLevel, PinState, strActive, strInactive, "L", "H", ShortHoldMilliseconds) { ; };
  clsPin(uint8_t Pin, uint8_t Mode, uint8_t ActiveLevel, uint8_t PinState, String strActive, String strInactive) : clsPin(Pin, Mode, ActiveLevel, PinState, strActive, strInactive, 0) { ; };
  clsPin(uint8_t Pin, uint8_t Mode, uint8_t ActiveLevel, uint8_t PinState) : clsPin(Pin, Mode, ActiveLevel, PinState, "A", "a") { ; };
  clsPin(uint8_t Pin, uint8_t Mode, uint8_t ActiveLevel) : clsPin(Pin, Mode, ActiveLevel, false) { ; };
  void SetMode();
  ledc_timer_bit_t GetTimerResolution(uint32_t Frequency);

  void setupLedCpulseTiming(uint32_t Frequency, uint32_t PulseTimeUs, ledc_timer_t Timer, ledc_channel_t Channel, ledc_timer_bit_t TimerResolution);
  void setupLedCpulseTiming(uint32_t Frequency, uint32_t PulseTimeUs, ledc_timer_t Timer, ledc_channel_t Channel);
  void setupLedCpulseTiming(uint32_t Frequency, uint32_t PulseTimeUs);
  void setupLedCpulseTiming(uint32_t Frequency);
  void setupLedCdutyCycle(uint32_t Frequency, uint32_t DutyCycle, ledc_timer_t Timer, ledc_channel_t Channel);
  void setupLedCdutyCycle(uint32_t Frequency, uint32_t DutyCycle);
  void SetLedCpulseTime(uint32_t PulseTimeUs);

  void TogglePin();
  void Flash(uint8_t Count, uint16_t TimeActive, uint16_t TimeInactive);
  void Flash();
  void SetActive();
  void SetInactive();
  void SetState(uint8_t State);
  void SetLevel(uint8_t Level);
  bool PWMsignalActive(unsigned long LastMilliSeconds);
  bool PWMsignalActive();
  void SetupPWM(uint32_t Frequency, uint32_t ResolutionBits);
  void SetDutyCycle(uint32_t DutyCycle);
  void SetPulseTimeUs(uint32_t PulseTimeUs);

  uint8_t IRAM_ATTR GetState();
  uint8_t IRAM_ATTR GetLevel();
  String GetStateStr(String strActive, String strInactive);
  String GetStateStr();
  String GetLevelStr();
  String GetPinStateStr();

  IRAM_ATTR void ProcessPWMsignal();
  void AttachInterrupt(void (*ISR_callback)(void), uint8_t InterruptMode);
  void AttachInterrupt(void (*ISR_callback)(void));
  void AttachInterrupt();
  void IRAM_ATTR ProcessPinActiveISR();
  void IRAM_ATTR ProcessPinState(); // processes the pin changes to detect a long or short active time and pin changes
  bool IRAM_ATTR PinStateStable(long StableTime);
  bool IRAM_ATTR PinStateStable();
  PinStates IRAM_ATTR ProcessStateChange();
  PinStates IRAM_ATTR ProcessStateStableShort();
  PinStates IRAM_ATTR ProcessStateStable();
  PinStates IRAM_ATTR GetPinState();
  void IRAM_ATTR SetProcessed();

private:
};
