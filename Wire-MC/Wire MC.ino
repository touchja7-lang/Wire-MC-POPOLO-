// ============================================================
// TB6600 stepper: continuous forward/back loop with START/STOP
// button (LA38). Status is printed on Serial only (9600 baud).
//
//  - Press the button to start; press it again to stop.
//  - On stop, the carriage returns to Home (position 0).
//  - No dwell at either end.
//  - Short ramp at the start/end of every move to avoid lost steps.
//  - No Serial output while the motor is moving.
// ============================================================

// ---------------- Pins ----------------
constexpr uint8_t PIN_PUL    = 2;    // TB6600 PUL+
constexpr uint8_t PIN_DIR    = 3;    // TB6600 DIR+
constexpr uint8_t PIN_BUTTON = 4;    // LA38 button (INPUT_PULLUP)

constexpr int8_t  PIN_ENABLE   = -1;   // TB6600 ENA+, -1 = not wired
constexpr uint8_t ENABLE_LEVEL = LOW;  // level that enables the driver

// Level read while the button is pressed.
//   NC contact to GND with INPUT_PULLUP -> HIGH (current wiring)
//   NO contact to GND with INPUT_PULLUP -> LOW
constexpr uint8_t BUTTON_PRESSED_LEVEL = HIGH;

// ---------------- Mechanics ----------------
constexpr long  PULSES_PER_REV = 3200;   // 200 steps x 16 microsteps
constexpr float CM_PER_REV     = 0.8f;
constexpr float STROKE_CM      = 3.3f;
constexpr long  STROKE_STEPS   =
    static_cast<long>(STROKE_CM / CM_PER_REV * PULSES_PER_REV + 0.5f);

constexpr uint8_t DIR_FORWARD  = HIGH;
constexpr uint8_t DIR_BACKWARD = LOW;

// ---------------- Speed ----------------
// Values are microseconds for HIGH and for LOW, so the step period is 2x.
// 20 us is very fast and can lose steps on a 12 V supply under load;
// if the motor stalls or drifts, try 40 or 60.
constexpr unsigned int CRUISE_US     = 20;    // working speed
constexpr unsigned int HOMING_US     = 100;   // homing speed (slower, safer)
constexpr unsigned int RAMP_START_US = 100;   // speed at the very start/end of a move
constexpr long         RAMP_STEPS    = 400;   // 0 = no ramp

// ---------------- Button timing ----------------
constexpr unsigned long DEBOUNCE_MS     = 30;  // START press / release debounce
constexpr unsigned long STOP_CONFIRM_MS = 25;  // STOP must be held this long while running

// ---------------- State ----------------
long          currentSteps = 0;      // steps from Home
unsigned long cycleCount   = 0;
bool          running      = false;

// ============================================================
// Button helpers
// ============================================================
inline bool buttonDown() {
  return digitalRead(PIN_BUTTON) == BUTTON_PRESSED_LEVEL;
}

// True only if the button stays pressed for the whole period (blocking).
bool buttonHeldFor(unsigned long ms) {
  const unsigned long start = millis();
  while (millis() - start < ms) {
    if (!buttonDown()) return false;
  }
  return true;
}

// Blocks until the button has been released for DEBOUNCE_MS.
void waitForRelease() {
  unsigned long releasedSince = millis();
  for (;;) {
    if (buttonDown()) {
      releasedSince = millis();
    } else if (millis() - releasedSince >= DEBOUNCE_MS) {
      return;
    }
  }
}

// Idle-state START detection: debounced press, then waits for release.
bool startPressed() {
  if (!buttonDown()) return false;
  if (!buttonHeldFor(DEBOUNCE_MS)) return false;
  waitForRelease();
  return true;
}

// ============================================================
// Motor helpers
// ============================================================
void setDriverEnabled(bool enabled) {
  if (PIN_ENABLE < 0) return;
  digitalWrite(PIN_ENABLE, enabled ? ENABLE_LEVEL : !ENABLE_LEVEL);
}

// Half-period for step 'i' of 'total': RAMP_START_US at both ends,
// falling linearly to 'cruiseUs' after RAMP_STEPS steps.
unsigned int stepDelayUs(long i, long total, unsigned int cruiseUs) {
  if (RAMP_STEPS <= 0) return cruiseUs;

  const long fromEdge = min(i, total - 1 - i);
  if (fromEdge >= RAMP_STEPS) return cruiseUs;

  const long slowdown = (static_cast<long>(RAMP_START_US) - cruiseUs)
                        * (RAMP_STEPS - fromEdge) / RAMP_STEPS;
  return static_cast<unsigned int>(cruiseUs + slowdown);
}

// Moves 'steps' pulses in direction 'dir' and updates currentSteps.
// With allowStop, a press held for STOP_CONFIRM_MS aborts the move.
// Returns true if the whole move completed.
bool moveSteps(long steps, uint8_t dir, unsigned int cruiseUs, bool allowStop) {
  if (steps <= 0) return true;

  digitalWrite(PIN_DIR, dir);
  delayMicroseconds(5);                       // TB6600 DIR setup time

  const int increment = (dir == DIR_FORWARD) ? 1 : -1;
  bool pressing = false;
  unsigned long pressedSince = 0;

  for (long i = 0; i < steps; i++) {
    if (allowStop) {
      if (buttonDown()) {
        if (!pressing) {
          pressing = true;
          pressedSince = millis();
        } else if (millis() - pressedSince >= STOP_CONFIRM_MS) {
          return false;
        }
      } else {
        pressing = false;
      }
    }

    const unsigned int us = stepDelayUs(i, steps, cruiseUs);
    digitalWrite(PIN_PUL, HIGH);
    delayMicroseconds(us);
    digitalWrite(PIN_PUL, LOW);
    delayMicroseconds(us);

    currentSteps += increment;
  }
  return true;
}

void returnHome() {
  if (currentSteps == 0) {
    Serial.println(F("-> Already at Home (0)."));
    return;
  }

  const uint8_t homeDir = (currentSteps > 0) ? DIR_BACKWARD : DIR_FORWARD;
  const long stepsToHome = labs(currentSteps);

  Serial.print(F("-> Homing... steps: "));
  Serial.println(stepsToHome);

  moveSteps(stepsToHome, homeDir, HOMING_US, false);   // not interruptible

  Serial.print(F("[SUCCESS] Motor returned to Home. Position: "));
  Serial.println(currentSteps);
}

void stopAndHome() {
  running = false;

  Serial.print(F("[STOP] Cycles completed: "));
  Serial.println(cycleCount);
  Serial.println(F("[STOP] Returning to Home..."));

  waitForRelease();            // the STOP press must be released first
  returnHome();
  setDriverEnabled(false);
  waitForRelease();            // ignore any press made while homing

  Serial.println(F("-> Ready. Press button to START."));
}

// ============================================================
// Arduino
// ============================================================
void setup() {
  Serial.begin(9600);

  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  if (PIN_ENABLE >= 0) pinMode(PIN_ENABLE, OUTPUT);

  digitalWrite(PIN_PUL, LOW);
  digitalWrite(PIN_DIR, LOW);
  setDriverEnabled(false);

  delay(2000);                 // let the power settle (motor is not running)

  Serial.println(F("========================================="));
  Serial.println(F("System Initialized (Continuous Loop Mode)"));
  Serial.print(F("Steps per stroke: "));
  Serial.println(STROKE_STEPS);
  Serial.println(F("-> Ready. Press button to START."));
  Serial.println(F("========================================="));
}

void loop() {
  if (!running) {
    if (startPressed()) {
      running = true;
      cycleCount = 0;
      setDriverEnabled(true);
      Serial.println(F("[START] Continuous loop started..."));
    }
    return;
  }

  // Forward then backward, with no pause in between.
  if (!moveSteps(STROKE_STEPS, DIR_FORWARD, CRUISE_US, true) ||
      !moveSteps(STROKE_STEPS, DIR_BACKWARD, CRUISE_US, true)) {
    stopAndHome();
    return;
  }

  cycleCount++;                // printed when STOP is pressed
}
