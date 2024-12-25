#define FIRMWARE_VERSION "Servo knob: 1.1"
#include <Arduino.h>
#include "clsPin.h"
#include "clsGeneral.h"
#include <U8g2lib.h>
#include "u8g2TextBox.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <Preferences.h>
#include "esp_err.h"
#include "AiEsp32RotaryEncoder.h"

#ifdef NoRGB // if the board doesn't match the selected board, the RGB support could prevent correctly setting an output pin
#undef RGB_BUILTIN
#endif

#ifndef ROTARY_DEBOUNCE_TIME
#define ROTARY_DEBOUNCE_TIME 100
#endif

#ifndef PWM_DEBOUNCE_TIME
#define PWM_DEBOUNCE_TIME 10
#endif
#define SERVO_FREQUENCY 50

enum Modes // Used to switch between setup and run
{
  ModeRun = 0,
  ModeSetupRPMstep = 1,
  ModeSetupRPMmin = 2,
  ModeSetupRPMmax = 3,
  ModeSetupServoMin = 4,
  ModeSetupServoMax = 5,
  ModeSetupServoDeadZone = 6,
  ModeSetupLimitInfo = 7,
  ModeReverseKnob = 8
};
Modes const FirstMode = ModeRun;
Modes const LastMode = ModeReverseKnob;
// Sequence must be equal to enum Modes cequence
StorageDataLong Setting[] = {
    StorageDataLong("RPM", "RPM", "RPM", RPM_MIN, RPM_MIN, RPM_MAX, 10, ""),
    StorageDataLong("RPMstep", "RPMstep", "RPM step", 10, 1, 500, 1, ""),
    StorageDataLong("RPMmin", "RPMmin", "Minimum RPM", RPM_MIN, RPM_MIN, RPM_MAX, 10, ""),
    StorageDataLong("RPMmax", "RPMmax", "Maximum RPM", RPM_MAX, RPM_MIN, RPM_MAX, 10, ""),
    StorageDataLong("SerMin", "ServoMin", "Minimum Angle", SERVO_ANGLE_MIN, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, 1, "°"),
    StorageDataLong("SerMax", "ServoMax", "Maximum Angle", SERVO_ANGLE_MAX, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, 1, "°"),
    StorageDataLong("SerDb", "ServoDeadBand", "Servo dead band", SERVO_DEAD_BAND_MIN, SERVO_DEAD_BAND_MIN, SERVO_DEAD_BAND_MAX, 1, "°"),
    StorageDataLong("LimitInfo", "LimitInfo", "Limit Info", 0, 0, 1, 0, ""),
    StorageDataLong("ReverseKnob", "ReverseKnob", "Reverse Knob", 0, 0, 1, 0, "")};

// Addition constructors for other types of displays can be found at
// https://github.com/olikraus/u8g2/wiki/u8g2setupcpp#buffer-size

// Tested OK ESP32C3-mini pins 6,7;5,7;3,4 || hardware i2c using Wire.begin(SDA,SCL) before u8g2.begin()
U8G2_SH1106_128X64_NONAME_F_HW_I2C OledDisplay(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C OledDisplay(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

u8g2TextBox Version(&OledDisplay, 0, 0, 128, 8, u8g2_font_5x8_mr, "", FIRMWARE_VERSION, false, 3, 1);

u8g2TextBox PinsState(&Version, 0, 0, 96, 8, "PinState: ", "");
u8g2TextBox OnState(&PinsState, 96, 0, 24, 8, "", "");
u8g2TextBox AliveState(&OnState, 24, 0, 8, 8, "", "");
u8g2TextBox Setpoint(&PinsState, 0, 8, 128, 8, "Setpoint:  ", " RPM");
u8g2TextBox ServoPosition(&Setpoint, 0, 8, "Angle:      ", ""); // Angle sign ° is not supported
u8g2TextBox PWMfrequency(&ServoPosition, 0, 8, 64, 8, "PWM: ", " Hz");
u8g2TextBox PWMdutyCycle(&PWMfrequency, 64, 0, " ", " %");
u8g2TextBox Counter(&PWMfrequency, 0, 8, 48, 8, "Cnt: ", "");
u8g2TextBox Debug(&Counter, 48, 0, 80, 8, "", "");
u8g2TextBox RPMlarge(&Counter, 0, 8, 128, 32, u8g2_font_spleen16x32_mr, "RPM ", "");

u8g2TextBox SetupTitle(&OledDisplay, 0, 0, 128, 16, u8g2_font_9x15B_mr, "Setup", "", false, 3, 1);
u8g2TextBox Description(&SetupTitle, 0, 24, 128, 8, u8g2_font_5x8_mr, "", "");
u8g2TextBox SetupSettinglarge(&Description, 0, 16, 128, 32, u8g2_font_spleen16x32_mr, "", "");

AiEsp32RotaryEncoder RotaryEncoderCW = AiEsp32RotaryEncoder(ROTARY_A_GPIO, ROTARY_B_GPIO, ROTARY_SW_GPIO, ROTARY_VCC_P, ROTARY_STEPS);
AiEsp32RotaryEncoder RotaryEncoderCCW = AiEsp32RotaryEncoder(ROTARY_B_GPIO, ROTARY_A_GPIO, ROTARY_SW_GPIO, ROTARY_VCC_P, ROTARY_STEPS);
AiEsp32RotaryEncoder RotaryEncoder;

Modes Mode = ModeRun;
long RPMsetpoint = 0;
long ServoPulseTime = 0; // usec
long ServoAngle = 0;     // °
bool ServoActive = false;

volatile bool SettingChanged = false;
volatile unsigned long SettingChangedAt = 0;
volatile bool SettingUpdateRequest = false;
volatile bool SettingUpdateRequestNow = false;

volatile int32_t DebugCounter = 0;
String DebugString = "--";

Preferences FlashData;

clsPin PinLed(LED_GPIO, OUTPUT, LED_ACTIVE, false, "L", "l");
clsPin PinRotaryA(ROTARY_A_GPIO, INPUT_PULLUP, ROTARY_ACTIVE, false, "A", "a");
clsPin PinRotaryB(ROTARY_B_GPIO, INPUT_PULLUP, ROTARY_ACTIVE, false, "B", "b");
clsPin PinRotarySW(ROTARY_SW_GPIO, INPUT_PULLUP, ROTARY_ACTIVE, false, "Sw", "sw", 2000);
clsPin PinServo(SERVO_GPIO, OUTPUT, SERVO_ACTIVE, false, "Se", "se");
clsPin PinPWM(PWM_GPIO, INPUT_PULLUP, PWM_ACTIVE, false, "P", "p");

String GetOnStateStr()
{
  if (ServoActive)
    return "ON";
  else
    return "off";
}
void ShowOnState()
{
  OnState.Show(GetOnStateStr());
}
void ShowSetupTitle()
{
  SetupTitle.Show();
}
void ShowPinState()
{
  String S = "";
  S = S + PinRotaryA.GetStateStr();
  S = S + PinRotaryB.GetStateStr();
  S = S + PinRotarySW.GetStateStr();
  S = S + PinPWM.GetStateStr();
  S = S + PinServo.GetStateStr();
  // S = S + PinLed.GetStateStr();
  PinsState.Show(S);
}
void ShowAlive()
{
  AliveState.ShowAlive();
}
void ShowSetpoint()
{
  Setpoint.Show(Setting[Mode].Value);
}
void ShowCounter()
{
  Counter.Show(DebugCounter);
}
void ShowRPM()
{
  if (ServoActive)
    RPMlarge.Show(Setting[ModeRun].Value);
  else
    RPMlarge.Show(0);
}
void ShowServoPosition()
{
  ServoPosition.Show(ServoAngle);
}
void ShowPWMinfo()
{
  PWMfrequency.Show(PinPWM.GetPWMfrequency());
  PWMdutyCycle.Show((long)PinPWM.GetPWMdutyCycle());
}
void ShowDebug()
{
  Debug.Show(DebugString);
}
void ShowDescription()
{
  Description.Show(Setting[Mode].Description);
}
void ShowSetupSetting()
{
  SetupSettinglarge.Show((String)Setting[Mode].Value, "", "");
}
void ShowStatusSetup(bool FullInfo)
{
  ShowSetupTitle();
  ShowAlive();
  ShowDescription();
  ShowSetupSetting();
}
void ShowStatusRun(bool FullInfo)
{
  if (FullInfo)
    ShowPinState();
  if (FullInfo)
    ShowOnState();
  ShowAlive();
  ShowSetpoint();
  if (FullInfo)
    ShowServoPosition();
  ShowRPM();
  if (FullInfo)
  {
    ShowPWMinfo();
  }
  if (FullInfo)
  {
    ShowCounter();
    ShowDebug();
  }
}
void ShowStatus()
{
  bool FullInfo = Setting[ModeSetupLimitInfo].Value == 0;
  switch (Mode)
  {
  case ModeRun:
    ShowStatusRun(FullInfo);
    break;
  default: // Must be in setup mode
    ShowStatusSetup(FullInfo);
    break;
  }
}
void ReadEncoderValue()
{
  switch (Mode)
  {
  case ModeRun:
  case ModeSetupRPMmin:
  case ModeSetupRPMmax:
    Setting[Mode].Value = RotaryEncoder.readEncoder() * Setting[ModeSetupRPMstep].Value; // get the encoder current value
    break;
  default:
    Setting[Mode].Value = RotaryEncoder.readEncoder(); // get the encoder current value
    break;
  }
}
long CalculateServoRPM(long DutyCycle)
{
  if (DutyCycle == 0)
    return 0;
  else
    return (long)MapValue(DutyCycle, 0, 100, Setting[ModeSetupRPMmin].Value, Setting[ModeSetupRPMmax].Value); // Calculate the RPM
}
long CalculateServoAngle(long RPM, long RPMmin, long RPMmax, long ServoAngleMin, long ServoAngleMax)
{
  RPM = AdjustValue(RPMmin, RPMmax, RPM);
  long Angle = (long)MapValue(RPM, RPMmin, RPMmax, ServoAngleMin, ServoAngleMax);
  Angle = AdjustValue(ServoAngleMin, ServoAngleMax, Angle);
  return Angle;
}
long CalculateServoPulseTime(long Angle, long AngleMin, long AngleMax)
{
  Angle = AdjustValue(AngleMin, AngleMax, Angle);                     // bring the angle within limits
  long PulseTime = (long)MapValue(Angle, 0, 180, AngleMin, AngleMax); // calculate the pulse time for of a 180° servo
  return PulseTime;
}
long CalculateServoPulseTime(long Angle)
{
  Angle = AdjustValue(Setting[ModeSetupServoMin].Value, Setting[ModeSetupServoMax].Value, Angle); // bring the angle within limits
  long PulseTime = (long)MapValue(Angle, 0, 180, 500, 2500);                                      // calculate the pulse time for of a 180° servo
  return PulseTime;
}
void CalculateServoAngleAndPulseTime(long RPM)
{

  if (RPM == -1) // This is a request to shutoff the spindle
  {
    ServoAngle = Setting[ModeSetupServoMin].Value; // set the servo at the min position
    ServoActive = false;
  }
  else
  {
    ServoAngle = CalculateServoAngle(RPM, Setting[ModeSetupRPMmin].Value, Setting[ModeSetupRPMmax].Value, Setting[ModeSetupServoMin].Value, Setting[ModeSetupServoMax].Value);
  }
  ServoPulseTime = CalculateServoPulseTime(ServoAngle);
}
void CalculateServoPosition()
{
  switch (Mode)
  {
  case ModeRun: // the encoder value defines the RPM
    if (ServoActive)
      CalculateServoAngleAndPulseTime(Setting[ModeRun].Value); // when active, set selected RPM
    else
      CalculateServoAngleAndPulseTime(Setting[ModeSetupRPMmin].Value); // when inactive, set lowest RPM
    break;
  case ModeSetupServoMin:                                            // the encoder value defines the Position
    CalculateServoAngleAndPulseTime(Setting[ModeSetupRPMmin].Value); // when inactive, set lowest RPM
    break;
  case ModeSetupServoMax:
    CalculateServoAngleAndPulseTime(Setting[ModeSetupRPMmax].Value); // when inactive, set lowest RPM
    break;
  default:
    CalculateServoAngleAndPulseTime(Setting[ModeSetupRPMmin].Value); // when inactive, set lowest RPM
    break;
    ;
  }
}
void IRAM_ATTR SignalSettingChanged()
{
  SettingChanged = true;
  SettingChangedAt = millis();
  SettingUpdateRequest = true;
}
void IRAM_ATTR readEncoderISR()
{
  RotaryEncoder.readEncoder_ISR();
  if (RotaryEncoder.encoderChanged())
    SignalSettingChanged(); // Signal that a setting has changed
}
void SetServoActive(bool Active)
{
  ServoActive = Active;
}
void ToggleServoActive()
{
  SetServoActive(!ServoActive);
}
void SetupRotaryEncoder(Modes Mode)
{
  if (Setting[ModeReverseKnob].Value == 0) // Select the right rotary encoder
    RotaryEncoder = RotaryEncoderCW;
  else
    RotaryEncoder = RotaryEncoderCCW;
  RotaryEncoder.begin();
  RotaryEncoder.setup(readEncoderISR);
  switch (Mode)
  {
  case ModeRun:
    RotaryEncoder.setBoundaries(Setting[ModeSetupRPMmin].Value / Setting[ModeSetupRPMstep].Value, Setting[ModeSetupRPMmax].Value / Setting[ModeSetupRPMstep].Value, false);
    RotaryEncoder.setEncoderValue(Setting[Mode].Value / Setting[ModeSetupRPMstep].Value);
    RotaryEncoder.setAcceleration(ROTARY_ACCELERATION_MODE_RUN); // or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
    break;
  case ModeSetupRPMmin:
  case ModeSetupRPMmax:
    RotaryEncoder.setBoundaries(Setting[Mode].MinValue / Setting[ModeSetupRPMstep].Value, Setting[Mode].MaxValue / Setting[ModeSetupRPMstep].Value, false);
    RotaryEncoder.setEncoderValue(Setting[Mode].Value / Setting[ModeSetupRPMstep].Value);
    RotaryEncoder.setAcceleration(ROTARY_ACCELERATION_MODE_RUN); // or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
    break;
  default:
    RotaryEncoder.setBoundaries(Setting[Mode].MinValue, Setting[Mode].MaxValue, false); // minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    RotaryEncoder.setEncoderValue(Setting[Mode].Value);
    RotaryEncoder.setAcceleration(ROTARY_ACCELERATION_MODE_SETUP); // or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
    break;
  }
}
void SetupServo()
{
  PinServo.setupLedCpulseTiming(SERVO_FREQUENCY, 0); // Setup the pulse timing and set disabled
  SetServoActive(false);
}
void ReadSettings()
{
  for (int i = 0; i <= LastMode; i++)
  {
    Setting[i].Value = FlashData.getLong(Setting[i].StorageID.c_str(), Setting[i].Value);
  }
  // DebugString = Setting[ModeRun].Value;
}
void WriteSettings()
{
  for (int i = 0; i <= LastMode; i++)
  {
    FlashData.putLong(Setting[i].StorageID.c_str(), Setting[i].Value);
  }
  // DebugString = Setting[ModeRun].Value;
}
void SetInactive()
{
  SetServoActive(false);
}
void SetActive()
{
  SetServoActive(true);
}
void ToggleActive()
{
  SetServoActive(!ServoActive);
}
void ReadWriteFlashData(bool read)
{
  long LongValue = 0;
  String DBversion = "1.0";
  if (read)
  {
    DBversion = FlashData.getString("DBversion", DBversion);
    ReadSettings(); // read the settings for this mode
  }
  else
  {
    FlashData.putString("DBversion", "1.0");
    WriteSettings();
    DebugCounter++;
  }
}
void SetMode(Modes NewMode, bool Force)
{
  if (Mode == NewMode) // nothing to change, starting up or entering setup
    if (!Force)
      return;
  Mode = NewMode;           // Set the new mode
  SetupRotaryEncoder(Mode); // Setup the rotary encoder
  SetInactive();            // Set servo inactive when changing mode
}
void SetMode(Modes NewMode)
{
  SetMode(NewMode, false);
}
void ProcessUpdateRequestNow()
{
  if (SettingUpdateRequestNow == false)
    return;
  SettingChanged = false;
  SettingUpdateRequestNow = false;
  ReadWriteFlashData(false);
}
void RequestUpdateNow()
{
  SettingUpdateRequestNow = true; // Signal to update a.s.p.
}
void ProcessSetpointChanged()
{
  if (!SettingChanged)
    return;           // Nothing changed
  ReadEncoderValue(); // get the encoder current value
  switch (Mode)
  {
  case ModeRun:
  case ModeSetupRPMmin:
  case ModeSetupRPMmax:
  case ModeSetupServoMin:
  case ModeSetupServoMax:
    CalculateServoPosition();
    break;
  }
  if (Mode == ModeRun) // In run mode update after 5 sec inactivity
  {
    if (millis() > (SettingChangedAt + 5000)) // Request for update when stable
      RequestUpdateNow();
  }
}
void SetupDebouncing()
{
  PinRotarySW.DebounceTime = ROTARY_DEBOUNCE_TIME;
}
void SetNextMode()
{
  switch (Mode)
  {
  case LastMode:
    SetMode(FirstMode);
    break;
  default:
    Modes NextMode = (Modes)((int16_t)Mode + 1); // select the next mode
    SetMode(NextMode);                           // set the mode
    break;
  }
  Version.ClearDisplay(); // clear the dislay to remove all previous data
}

void ProcessPWMsignal()
{
  static long LastRPM = 0;
  long Delta = MaxValue(Setting[ModeSetupRPMstep].Value / 2, Setting[ModeSetupServoDeadZone].Value);
  switch (Mode)
  {
  case ModeRun:
    if (PinPWM.PulseProcessed == true) // if there is a new pulse signal to process
    {
      long DutyCycle = PinPWM.GetPWMdutyCycle(); // Get the pulse dutyclycle
      long RPM = CalculateServoRPM(DutyCycle);   // Calculate the RPM
      if (RPM != LastRPM)                        // only process a change once so it can be overruled
        if (!ValueInRange(LastRPM - Delta, LastRPM + Delta, RPM))
        {
          if (RPM == 0)
          {
            RPM = -1;
            SetInactive(); // Inactivate servo
          }
          else
          {
            Setting[ModeRun].Value = RPM; // Set the new RPM setpoint
            // SetupRotaryEncoder(ModeRun);
            // RotaryEncoder.setEncoderValue(RPM);
            SetActive(); // Activate the servo
          }
          CalculateServoAngleAndPulseTime(RPM);
          LastRPM = RPM;
          // Setting[ModeRun].Value=RPM;
        }
    }
    else if (!PinPWM.PWMsignalActive())
    {
      CalculateServoAngleAndPulseTime(-1); // Shutoff the servo
    }
    PinPWM.PulseProcessed == false; // Prepare for a new pulse;
  }
}
void ProcessServoPosition()
{
  static unsigned long DeactivateAtTime = millis() + SERVO_DEACTIVATE_DELAY; // Set timer for deactivating servo
  static uint32_t LastServoPulseTime = -1;
  static bool LastServoActive = ServoActive;
  static bool ActivateServo = true;            // at startup, set the correct position
  if (ActivateServo == true)                   // if the servo has not to be disabled
    PinServo.SetLedCpulseTime(ServoPulseTime); // Set the pulse time, shut off time not reached
  else
    PinServo.SetLedCpulseTime(CalculateServoPulseTime(Setting[ModeSetupServoMin].Value)); // Set the min
  // now check if the servo can be disabled to preserve servo wear
  if ((LastServoPulseTime != ServoPulseTime) || (ServoActive != LastServoActive)) // new setting or change in active, reset timer
  {
    DeactivateAtTime = millis() + SERVO_DEACTIVATE_DELAY;
    LastServoPulseTime = ServoPulseTime; // preserve the last pulse time
    LastServoActive = ServoActive;
    ActivateServo = true; // signal to set the servo active at the next pass
  }
  if (millis() > DeactivateAtTime) // if the servo motor has to be disabled to
  {
    // ActivateServo = false;
    ServoPulseTime = 0;
  }
  PinServo.SetLedCpulseTime(ServoPulseTime); // Set the servo pulse time
}
void SetServoInStartPosition()
{
  CalculateServoAngleAndPulseTime(Setting[ModeSetupServoMin].Value); // At the first pass, set the servo in the start position
  ProcessServoPosition();
}
void ProcessButtonActiveToggle()
{
  ToggleActive(); // ToggleActive
  CalculateServoPosition();
}
void ProcessRotaryButtonUpdateRequest()
{
  switch (PinRotarySW.GetPinState()) // update the SW button status
  {
    // process SW pressed on released
  case PinStateIU: // Button was pressed, comming from unprocessed active
    PinRotarySW.SetProcessed();
    switch (Mode)
    {
    case ModeRun:                  // In run mode
      ProcessButtonActiveToggle(); // toggle the active state
      break;
    default:              // Whe are in setup mode and the mode is changed so save value
      ReadEncoderValue(); // Read the current data
      if (Mode == LastMode)
        RequestUpdateNow(); // Request to update current value a.s.p.
      SetNextMode();        // Select the next mode
      break;
    }
    break;
  case PinStateS: // Button was pressed for a short time, so select the setup mode
    PinRotarySW.SetProcessed();
    switch (Mode)
    {
    case ModeRun:    // In run mode
      SetInactive(); // When changing modes, set servo to inactive
      SetServoInStartPosition();
      SetNextMode(); // Select the first setup mode
      break;
    }
    break;
  }
}
void IRAM_ATTR ProcessPWMsignalISR() // This code analyses the PWM signal by calling the Pins PWM analyser
{
  PinPWM.ProcessPWMsignal();
}
void TestPWM()
{
  int32_t t = 1500;                      // mid position
  int32_t f = SERVO_FREQUENCY;           // 50 Hz
  int32_t min = 1000;                    //-90°
  int32_t max = 2000;                    // 90°
  int32_t del = 4;                       // 4 usec each step
  int32_t delMid = 2000;                 // 2 seconds at mid position
  int32_t delEnd = 5000;                 // 5 seconds at end position
  PinPWM.setupLedCpulseTiming(f, min);   // 2048 resolution
  PinServo.setupLedCpulseTiming(f, min); // 2048 resolution
  for (int c = 1; c < 3; c++)
  {
    ShowStatus;
    for (int32_t i = min; i < max; i++)
    {
      PinPWM.SetLedCpulseTime(i);
      PinServo.SetLedCpulseTime(i);
      delay(del);
      if (i == t)
        delay(delMid);
    }
    PinPWM.SetLedCpulseTime(0);
    PinServo.SetLedCpulseTime(0);
    delay(delEnd);
    for (int32_t i = max; i > min; i--)
    {
      PinPWM.SetLedCpulseTime(i);
      PinServo.SetLedCpulseTime(i);
      delay(del);
      if (i == t)
        delay(delMid);
    }
    PinPWM.SetLedCpulseTime(0);
    PinServo.SetLedCpulseTime(0);
    delay(delEnd);
  }
}
void setup(void)
{
  // Serial.begin(115200);  // Can't use serial because rx/tx pins are used
  PinLed.Flash(); // #1 Show startingup
  FlashData.begin("ServoKnob", false);
  ReadWriteFlashData(true);
  PinLed.Flash();         // #2 Show progress
  SetMode(ModeRun, true); // The first time, force setting up the rotary encoder
  SetupServo();
  PinLed.Flash(); // #3 Show progress and give powersupply time to get stable
                  // Seems that pinmodes are changed after i2c is activated.
  Wire.begin(SDA_PIN, SCL_PIN);
  OledDisplay.begin(); // InjectionSetpoint = 0;// disable injection
  Version.Show();      // Show the version
  SetupDebouncing();
  PinPWM.AttachInterrupt(ProcessPWMsignalISR); // Attach the PWM input pin to an interrupt handler
  PinLed.Flash();                              // # 5 Show progress
  Version.ClearDisplay();
  ShowStatus();
  SetServoInStartPosition();
}

void loop(void)
{
  static bool FirstRun = true;
  unsigned long TimeTimeChanged = micros();
  // TestPWM();
  ShowStatus();
  ProcessPWMsignal();                 // in CNC mode, process the incoming PWM signal
  ProcessSetpointChanged();           // Process the change of a setpoint, may overrule PWM signal
  ProcessRotaryButtonUpdateRequest(); // Process the button pressed, may overrule PWM
  if (FirstRun)                       // Start inactive
  {
    SetInactive();
    CalculateServoAngleAndPulseTime(-1);
    FirstRun = false;
  }
  ProcessServoPosition();
  ProcessUpdateRequestNow(); // process the update request without delay
  //  PinLed.TogglePin();
  delay(10); // do a small delay so back ground tasks have more time to run
}
