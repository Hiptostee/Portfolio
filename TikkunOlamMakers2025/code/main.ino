/**
 * @file main.ino
 * @brief Controls a stepper motor with a joystick and features an adjustable zero position.
 *
 * This code allows for precise control of a stepper motor using a joystick.
 * It has two modes of operation, toggled by the joystick's built-in button:
 *
 * 1. RUN MODE:
 * - The motor's movement is restricted between a defined lower and upper 50bound.
 * - The position is tracked and printed to the Serial Monitor.
 *
 * 2. ADJUST MODE:
 * - The motor can move freely without any software limits.
 * - The Arduino's built-in LED will blink periodically to indicate this mode is active.
 * - The position is not tracked. This mode is used to manually move the motor
 * to a physical starting point or "zero" position.
 *
 * Switching from ADJUST MODE back to RUN MODE sets the current physical position
 * of the motor as the new logical zero (0) for position tracking.
 *
 * Hardware:
 * - Arduino or compatible microcontroller
 * - TB6600 Stepper Motor Driver
 * - Stepper Motor
 * - Joystick with a push-button
 */

// --- Pin Macros ---
// TB6600 Driver Pins
#define STEP_PIN 2   // PUL+ pin
#define DIR_PIN 3    // DIR+ pin
#define EN_PIN 4     // ENA+ pin
#define BUZZER_PIN 9 // buzzer Pin

// Joystick Pins
#define JOYSTICK_X_PIN A0  // VRx pin for X-axis movement
#define JOYSTICK_BTN_PIN 7 // SW pin for the push-button

// --- Control Variables ---
int _stepDelay = 0;                 // Delay in microseconds between steps (lower is faster)
long _currentPosition = 0;          // Tracks the motor's position in steps during RUN MODE
int _joystickValue = 0;             // Raw analog value from the joystick
bool _adjustMode = false;           // Start in RUN MODE by default
bool _buttonPreviouslyPressed = false;        // Was the button previously pressed
bool _beep = false;
unsigned long _lastBeepTime = 0;

// --- Constants ---
const int centerThreshold = 50;       // Joystick deadzone to prevent drift
const long lowerBound = 0;            // The minimum position in RUN MODE
const long upperBound = 15000;        // The maximum position in RUN MODE (135 degrees)
const long StepDelayMin = 50;         // Value in microseconds  
const long StepDelayMax = 500;        // Value in microseconds
const long JoystickCenterValue = 512; // When at neutral the joystick should read in 512
const long BeepDelay = 500;           // Length in time of the beeps in milliseconds
const bool PrintInfo = false;         // Print to Serial Monitor

// Helper pre-declarations
void Detect_AdjustMode();
void Handle_AdjustMode();
void HandleJoystick();
void DisplayPositionInformation();
void TakeStep(int stepDelay, bool clockwise);

void setup() {
  // Initialize serial communication for debugging
  if (PrintInfo)
  {
    Serial.begin(9600);
    Serial.println("Stepper motor controller starting up...");
  }

  // Configure pin modes
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(JOYSTICK_BTN_PIN, INPUT_PULLUP); // Use internal pull-up resistor for the button

  // Disable the driver by default (HIGH signal disables it on many common drivers)
  digitalWrite(EN_PIN, HIGH);
  
  if (PrintInfo)
  {
    Serial.println("✅ RUN MODE active. Position is bounded.");
  }
}

void loop() {
  Detect_AdjustMode();
  Handle_AdjustMode();
  HandleJoystick();
  DisplayPositionInformation();
}

void Detect_AdjustMode()
{
  bool buttonPressed = digitalRead(JOYSTICK_BTN_PIN) == LOW;
  if (buttonPressed && !_buttonPreviouslyPressed) 
  {
    _buttonPreviouslyPressed = true;
    _adjustMode = !_adjustMode; // Toggle the mode

    if (_adjustMode) 
    {
      Serial.println("\n⚙️ ADJUST MODE ON. Motor is now free. Move to your desired zero position and press the button.\n");
    } 
    else 
    {
      // When exiting adjust mode, reset the position to establish the new zero.
      _currentPosition = 0;
      if (PrintInfo)
      {
        Serial.println("\n✅ RUN MODE ON. Current position is now 0. Movement is bounded.\n");
      }
      _beep = false;
    }
  }
  else if (!buttonPressed)
  {
    _buttonPreviouslyPressed = false;
  }
  if (!_adjustMode)
  {
    noTone(BUZZER_PIN); 
  }
}

void Handle_AdjustMode()
{
  if (_adjustMode) {
    if (!_beep && (millis() - _lastBeepTime > BeepDelay)) {
      _lastBeepTime = millis();
      tone(BUZZER_PIN, 2000); // Send 1KHz sound signal
      _beep = true;
    }
    if (_beep && (millis() - _lastBeepTime > BeepDelay)) {
      _lastBeepTime = millis();
      noTone(BUZZER_PIN); 
      _beep = false;
    }
  }
}

void HandleJoystick()
{
  _joystickValue = analogRead(JOYSTICK_X_PIN) - JoystickCenterValue; // offset to account for center position. Possible values are between 0 and 1024
  if (abs(_joystickValue) > centerThreshold) {
    bool clockwise = (_joystickValue > 0);
    int abs_joystickValue = abs(_joystickValue);

    // Map the joystick's position to a step delay.
    // Further from center = smaller delay = faster speed.
    _stepDelay = map(abs_joystickValue, centerThreshold, JoystickCenterValue, StepDelayMax, StepDelayMin);

    if (_adjustMode) {
      TakeStep(_stepDelay, clockwise);
    } else {
      if ((clockwise && _currentPosition < upperBound) || (!clockwise && _currentPosition > lowerBound)) {
        TakeStep(_stepDelay, clockwise);
        _currentPosition += (clockwise ? 1 : -1);
      }
      else
      {
        if (digitalRead(EN_PIN) == LOW)
        {
          digitalWrite(EN_PIN, HIGH); // Disable motor when not moving
        }
      }
    }
  }
  else
  {
    if (digitalRead(EN_PIN) == LOW)
    {
      digitalWrite(EN_PIN, HIGH); // Disable motor when not moving
    }
  }
}

void DisplayPositionInformation()
{
  // Only print when the motor is stationary to prevent Serial commands
  // from interrupting the step pulses, which causes buggy rotation.
  static unsigned long lastPrint = 0;
  if (PrintInfo)
  {
    if (abs(_joystickValue) <= centerThreshold) {
      if (millis() - lastPrint > 250) {
        lastPrint = millis();
        Serial.print(_adjustMode ? "[ADJUST MODE] " : "[RUN MODE]    ");
        Serial.print("Position: ");
        if (_adjustMode) {
          Serial.println("UNTRACKED");
        } else {
          Serial.println(_currentPosition);
        }
      }
    }
  }
}

void TakeStep(int stepDelay, bool clockwise) 
{
  // Enable motor and set direction
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);

  // Create a pulse to trigger one step
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(stepDelay);
}
