#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// One TB6600 per motor. Default wiring is common-cathode:
// TB6600 PUL+/DIR+ -> Arduino pins; PUL-/DIR- -> Arduino GND.
constexpr uint8_t M1_STEP_PIN = 2;
constexpr uint8_t M1_DIR_PIN  = 5;
constexpr uint8_t M2_STEP_PIN = 3;
constexpr uint8_t M2_DIR_PIN  = 6;

constexpr long MOTOR_FULL_STEPS_PER_REV = 200;  // 17HS4401S: 1.8 deg
constexpr int MICROSTEPS = 16;                  // Measured TB6600 DIP: 1/16

// Motor turns needed for one output-shaft turn.
// Direct drive: 1.0. Reduction 4:1: 4.0 (not 0.25).
constexpr float M1_MOTOR_TURNS_PER_OUTPUT_TURN = 1.0f;
constexpr float M2_MOTOR_TURNS_PER_OUTPUT_TURN = 1.0f;

// Change to true if positive angle turns that motor the wrong way.
constexpr bool M1_REVERSE_DIRECTION = false;
constexpr bool M2_REVERSE_DIRECTION = false;
constexpr bool STEP_PULSE_ACTIVE_LOW = false;  // false = common-cathode PUL+ -> pin

// Lower speed/acceleration if a loaded motor stalls or loses steps.
constexpr float MAX_SPEED_STEPS_PER_SECOND = 500.0f;
constexpr float MAX_ACCELERATION_STEPS_PER_SECOND2 = 400.0f;
constexpr unsigned int MIN_STEP_PULSE_US = 10;
constexpr unsigned int DIR_SETUP_US = 5;
constexpr float MAX_ABS_ANGLE_DEG = 36000.0f;

AccelStepper motor1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

enum MotionState { IDLE, MOVE_ONCE, CYCLE, STOPPING };
MotionState motionState = IDLE;
long cycleOrigin[2];
long cycleAmplitude[2];
long cyclePosition[2];
unsigned long cycleStartUs = 0;
unsigned long cyclePeriodUs = 0;
bool cycleStopRequested = false;
bool cycleReturning = false;
unsigned long cycleLastStepUs[2];

char inputLine[48];
uint8_t inputLength = 0;
bool inputDiscard = false;
unsigned long lastInputMs = 0;

long angleToSteps(float angleDeg, float motorTurnsPerOutputTurn) {
  const float steps = angleDeg * MOTOR_FULL_STEPS_PER_REV * MICROSTEPS
                      * motorTurnsPerOutputTurn / 360.0f;
  return lroundf(steps);
}

void scheduleMove(long target1, long target2) {
  const long distance1 = labs(target1 - motor1.currentPosition());
  const long distance2 = labs(target2 - motor2.currentPosition());
  const long longestDistance = max(distance1, distance2);
  if (longestDistance == 0) return;

  // Scale speed and acceleration by distance so both axes share one move time.
  if (distance1 > 0) {
    const float scale1 = (float)distance1 / longestDistance;
    motor1.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND * scale1);
    motor1.setAcceleration(MAX_ACCELERATION_STEPS_PER_SECOND2 * scale1);
  }
  if (distance2 > 0) {
    const float scale2 = (float)distance2 / longestDistance;
    motor2.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND * scale2);
    motor2.setAcceleration(MAX_ACCELERATION_STEPS_PER_SECOND2 * scale2);
  }

  motor1.moveTo(target1);
  motor2.moveTo(target2);
}

bool isBusy() {
  return motionState != IDLE;
}

void startSingleMove(float angle1Deg, float angle2Deg) {
  const long steps1 = angleToSteps(angle1Deg, M1_MOTOR_TURNS_PER_OUTPUT_TURN);
  const long steps2 = angleToSteps(angle2Deg, M2_MOTOR_TURNS_PER_OUTPUT_TURN);

  if (steps1 == 0 && steps2 == 0) {
    Serial.println(F("OK,0,0"));
    return;
  }

  scheduleMove(motor1.currentPosition() + steps1, motor2.currentPosition() + steps2);
  motionState = MOVE_ONCE;
}

void startCycle(float angle1Deg, float angle2Deg) {
  const long steps1 = angleToSteps(angle1Deg, M1_MOTOR_TURNS_PER_OUTPUT_TURN);
  const long steps2 = angleToSteps(angle2Deg, M2_MOTOR_TURNS_PER_OUTPUT_TURN);

  if (steps1 == 0 && steps2 == 0) {
    Serial.println(F("ERR,cycle angle cannot both be zero"));
    return;
  }

  cycleOrigin[0] = motor1.currentPosition();
  cycleOrigin[1] = motor2.currentPosition();
  cycleAmplitude[0] = steps1;
  cycleAmplitude[1] = steps2;
  cyclePosition[0] = cycleOrigin[0];
  cyclePosition[1] = cycleOrigin[1];

  const long longestAmplitude = max(labs(steps1), labs(steps2));
  const float timeForSpeed = PI * longestAmplitude / MAX_SPEED_STEPS_PER_SECOND;
  const float timeForAcceleration = PI * sqrtf(
      2.0f * longestAmplitude / MAX_ACCELERATION_STEPS_PER_SECOND2);
  const float periodUs = max(timeForSpeed, timeForAcceleration) * 1000000.0f;
  if (!isfinite(periodUs) || periodUs < 1.0f || periodUs >= 4294967296.0f) {
    Serial.println(F("ERR,cycle period out of range"));
    return;
  }
  cyclePeriodUs = (unsigned long)periodUs;
  cycleStartUs = micros();
  cycleLastStepUs[0] = cycleLastStepUs[1] = cycleStartUs;
  cycleStopRequested = false;
  cycleReturning = false;
  motionState = CYCLE;
  Serial.println(F("CYCLE,smooth; send S to stop at origin"));
}

unsigned long cyclePhaseUs(unsigned long now) {
  unsigned long elapsed = now - cycleStartUs;
  if (elapsed >= cyclePeriodUs) {
    // Keep the anchor recent, so micros() rollover cannot reset the phase.
    cycleStartUs += (elapsed / cyclePeriodUs) * cyclePeriodUs;
    elapsed %= cyclePeriodUs;
    if (cycleStopRequested) cycleReturning = true;
  }
  return elapsed;
}

void stopMotion() {
  if (motionState == IDLE) {
    Serial.println(F("STOP"));
    return;
  }

  if (motionState == CYCLE) {
    if (!cycleStopRequested) {
      cyclePhaseUs(micros());
      cycleStopRequested = true;
    }
    Serial.println(F("STOPPING,returning to origin"));
    return;
  }

  // AccelStepper chooses a deceleration target using current acceleration.
  if (motionState != STOPPING) {
    motionState = STOPPING;
    motor1.stop();
    motor2.stop();
  }
  Serial.println(F("STOPPING"));
}

void setCycleDirection(uint8_t pin, bool positive, bool reverseDirection) {
  digitalWrite(pin, (positive ^ reverseDirection) ? HIGH : LOW);
}

void serviceCycle() {
  const unsigned long now = micros();
  const unsigned long phaseUs = cyclePhaseUs(now);
  const float phase = TWO_PI * (float)phaseUs / cyclePeriodUs;
  // Hold the origin after the stop boundary, even if a late loop missed it.
  const float progress = cycleReturning ? 0.0f : 0.5f * (1.0f - cosf(phase));
  const long target1 = cycleOrigin[0] + lroundf(cycleAmplitude[0] * progress);
  const long target2 = cycleOrigin[1] + lroundf(cycleAmplitude[1] * progress);
  const unsigned long minIntervalUs = (unsigned long)ceilf(1000000.0f / MAX_SPEED_STEPS_PER_SECOND);
  const bool step1 = target1 != cyclePosition[0] && now - cycleLastStepUs[0] >= minIntervalUs;
  const bool step2 = target2 != cyclePosition[1] && now - cycleLastStepUs[1] >= minIntervalUs;

  if (step1) setCycleDirection(M1_DIR_PIN, target1 > cyclePosition[0], M1_REVERSE_DIRECTION);
  if (step2) setCycleDirection(M2_DIR_PIN, target2 > cyclePosition[1], M2_REVERSE_DIRECTION);

  if (step1 || step2) {
    delayMicroseconds(DIR_SETUP_US);
    const uint8_t activeLevel = STEP_PULSE_ACTIVE_LOW ? LOW : HIGH;
    const uint8_t idleLevel = STEP_PULSE_ACTIVE_LOW ? HIGH : LOW;
    const unsigned long pulseUs = micros();
    if (step1) cycleLastStepUs[0] = pulseUs;
    if (step2) cycleLastStepUs[1] = pulseUs;
    if (step1) digitalWrite(M1_STEP_PIN, activeLevel);
    if (step2) digitalWrite(M2_STEP_PIN, activeLevel);
    delayMicroseconds(MIN_STEP_PULSE_US);
    if (step1) digitalWrite(M1_STEP_PIN, idleLevel);
    if (step2) digitalWrite(M2_STEP_PIN, idleLevel);
    if (step1) cyclePosition[0] += target1 > cyclePosition[0] ? 1 : -1;
    if (step2) cyclePosition[1] += target2 > cyclePosition[1] ? 1 : -1;
  }

  if (cycleReturning
      && cyclePosition[0] == cycleOrigin[0] && cyclePosition[1] == cycleOrigin[1]) {
    motor1.setCurrentPosition(cyclePosition[0]);
    motor2.setCurrentPosition(cyclePosition[1]);
    motionState = IDLE;
    Serial.println(F("STOP"));
  }
}

void serviceMotion() {
  if (motionState == IDLE) return;

  if (motionState == CYCLE) {
    serviceCycle();
    return;
  }

  // Do not combine these calls with ||: both motors must be serviced every loop.
  const bool motor1Running = motor1.run();
  const bool motor2Running = motor2.run();
  if (motor1Running || motor2Running) return;

  if (motionState == STOPPING) {
    motionState = IDLE;
    Serial.println(F("STOP"));
    return;
  }

  if (motionState == MOVE_ONCE) {
    motionState = IDLE;
    Serial.println(F("OK"));
  }
}

bool parseAngle(const char *text, float &value) {
  if (!text) return false;

  char *end = nullptr;
  value = strtod(text, &end);
  return end != text && *end == '\0' && isfinite(value)
         && fabs(value) <= MAX_ABS_ANGLE_DEG;
}

void handleLine(char *line) {
  char *command = strtok(line, " ,\t");
  if (!command) return;

  for (char *p = command; *p; ++p) *p = toupper((unsigned char)*p);

  if ((strcmp(command, "S") == 0 || strcmp(command, "STOP") == 0)
      && strtok(nullptr, " ,\t") == nullptr) {
    stopMotion();
    return;
  }

  if (strcmp(command, "H") == 0) {
    Serial.println(F("M a b=move once; C a b=repeat out/back; S=stop"));
    return;
  }

  const bool isMove = strcmp(command, "M") == 0;
  const bool isCycle = strcmp(command, "C") == 0;
  if (!isMove && !isCycle) {
    Serial.println(F("ERR,use: M 90 -45, C 90 90, or S"));
    return;
  }

  if (isBusy()) {
    Serial.println(F("ERR,busy; send S first"));
    return;
  }

  float angle1;
  float angle2;
  const bool valid = parseAngle(strtok(nullptr, " ,\t"), angle1)
                     && parseAngle(strtok(nullptr, " ,\t"), angle2)
                     && strtok(nullptr, " ,\t") == nullptr;

  if (!valid) {
    Serial.println(F("ERR,use: M 90 -45 or C 90 90"));
    return;
  }

  if (isMove) {
    startSingleMove(angle1, angle2);
  } else {
    startCycle(angle1, angle2);
  }
}

void setup() {
  motor1.setPinsInverted(M1_REVERSE_DIRECTION, STEP_PULSE_ACTIVE_LOW, false);
  motor2.setPinsInverted(M2_REVERSE_DIRECTION, STEP_PULSE_ACTIVE_LOW, false);
  motor1.setMinPulseWidth(MIN_STEP_PULSE_US);
  motor2.setMinPulseWidth(MIN_STEP_PULSE_US);
  motor1.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
  motor2.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
  motor1.setAcceleration(MAX_ACCELERATION_STEPS_PER_SECOND2);
  motor2.setAcceleration(MAX_ACCELERATION_STEPS_PER_SECOND2);

  Serial.begin(115200);
  Serial.println(F("READY: M a b; C a b; S"));
}

void loop() {
  // Bound command work so a continuous input stream cannot starve stepping.
  uint8_t budget = 32;
  while (budget-- && Serial.available()) {
    const char c = Serial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      inputLine[inputLength] = '\0';
      if (!inputDiscard) handleLine(inputLine);
      inputLength = 0;
      inputDiscard = false;
      lastInputMs = 0;
      continue;
    }

    lastInputMs = millis();
    if (inputDiscard) continue;
    if (((uint8_t)c < 32 || (uint8_t)c > 126) && c != '\t') {
      inputLength = 0;
      inputDiscard = true;
      Serial.println(F("ERR,invalid character"));
    } else if (inputLength < sizeof(inputLine) - 1) {
      inputLine[inputLength++] = c;
    } else {
      inputLength = 0;
      inputDiscard = true;
      Serial.println(F("ERR,line too long"));
    }
  }

  // Arduino Serial Monitor may be configured with "No line ending".
  // Treat 50 ms of silence as end of command so both settings work.
  if ((inputLength > 0 || inputDiscard) && !Serial.available() && millis() - lastInputMs >= 50) {
    inputLine[inputLength] = '\0';
    if (!inputDiscard) handleLine(inputLine);
    inputLength = 0;
    inputDiscard = false;
  }

  serviceMotion();
}
