#include <Arduino.h>

// Read A+ on an interrupt pin and B+ on a normal input: x1 decoding.
// Do not connect A-/B- to GND; insulate every unused differential wire.
constexpr uint8_t ENCODER1_A_PIN = 2;  // Mega PE4 / INT4
constexpr uint8_t ENCODER1_B_PIN = 4;  // Mega PG5
constexpr uint8_t ENCODER2_A_PIN = 3;  // Mega PE5 / INT5
constexpr uint8_t ENCODER2_B_PIN = 5;  // Mega PE3

constexpr long COUNTS_PER_REV = 2500;  // x1: one A rising edge per encoder line
constexpr int8_t ENCODER1_SIGN = 1;    // Change to -1 if direction is reversed.
constexpr int8_t ENCODER2_SIGN = 1;
constexpr unsigned long PRINT_INTERVAL_MS = 10;  // 100 Hz for IMU / encoder validation.

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
char commandLine[8];
uint8_t commandLength = 0;
bool commandOverflow = false;
bool timedOutput = true;
unsigned long sampleSequence = 0;

void onEncoder1A() {
  // Direct port read keeps ISR short. D4 is PG5 on Arduino Mega 2560.
  if (PING & _BV(PG5)) {
    --encoder1Count;
  } else {
    ++encoder1Count;
  }
}

void onEncoder2A() {
  // D5 is PE3 on Arduino Mega 2560.
  if (PINE & _BV(PE3)) {
    --encoder2Count;
  } else {
    ++encoder2Count;
  }
}

float angle0To360(long signedCount) {
  long wrapped = signedCount % COUNTS_PER_REV;
  if (wrapped < 0) wrapped += COUNTS_PER_REV;
  return wrapped * (360.0f / COUNTS_PER_REV);
}

void copyCounts(long &count1, long &count2, unsigned long &sampleUs) {
  const uint8_t savedSreg = SREG;
  noInterrupts();
  count1 = encoder1Count;
  count2 = encoder2Count;
  sampleUs = micros();  // Timestamp the atomic count snapshot, before serial output.
  SREG = savedSreg;
}

void zeroEncoder(uint8_t encoderNumber) {
  noInterrupts();
  if (encoderNumber == 0 || encoderNumber == 1) encoder1Count = 0;
  if (encoderNumber == 0 || encoderNumber == 2) encoder2Count = 0;
  interrupts();
}

void printData() {
  long raw1;
  long raw2;
  unsigned long sampleUs;
  copyCounts(raw1, raw2, sampleUs);

  const long count1 = raw1 * ENCODER1_SIGN;
  const long count2 = raw2 * ENCODER2_SIGN;

  if (timedOutput) {
    Serial.print(F("ENC,"));
    Serial.print(sampleSequence++);
    Serial.print(',');
    Serial.print(sampleUs);
    Serial.print(',');
  } else {
    Serial.print(millis());
    Serial.print(',');
    Serial.print(angle0To360(count1), 3);
    Serial.print(',');
    Serial.print(angle0To360(count2), 3);
    Serial.print(',');
  }
  Serial.print(count1);
  Serial.print(',');
  Serial.println(count2);
}

void printMetadata() {
  Serial.print(F("ENCODER,2,"));
  Serial.print(COUNTS_PER_REV);
  Serial.print(',');
  Serial.println(1000UL / PRINT_INTERVAL_MS);
}

void handleCommandLine(const char *line) {
  if (strcmp(line, "Z") == 0 || strcmp(line, "z") == 0) {
    zeroEncoder(0);
    Serial.println(F("ZERO,0"));
  } else if (strcmp(line, "Z1") == 0 || strcmp(line, "z1") == 0) {
    zeroEncoder(1);
    Serial.println(F("ZERO,1"));
  } else if (strcmp(line, "Z2") == 0 || strcmp(line, "z2") == 0) {
    zeroEncoder(2);
    Serial.println(F("ZERO,2"));
  } else if (strcmp(line, "P") == 0 || strcmp(line, "p") == 0) {
    printData();
  } else if (strcmp(line, "F") == 0 || strcmp(line, "f") == 0) {
    timedOutput = true;
    printMetadata();
  } else if (strcmp(line, "L") == 0 || strcmp(line, "l") == 0) {
    timedOutput = false;
    Serial.println(F("ms,encoder1_deg,encoder2_deg,encoder1_count,encoder2_count"));
  } else if (*line != '\0') {
    Serial.println(F("ERR,use Z, Z1, Z2, P, F or L"));
  }
}

void readCommands() {
  // Keep sampling even when the host continuously sends command bytes.
  uint8_t budget = 32;
  while (budget-- && Serial.available()) {
    const char c = Serial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      commandLine[commandLength] = '\0';
      if (!commandOverflow) handleCommandLine(commandLine);
      commandLength = 0;
      commandOverflow = false;
    } else if (commandOverflow) {
      continue;
    } else if (((uint8_t)c < 32 || (uint8_t)c > 126) && c != '\t') {
      commandLength = 0;
      commandOverflow = true;
      Serial.println(F("ERR,invalid character"));
    } else if (commandLength < sizeof(commandLine) - 1) {
      commandLine[commandLength++] = c;
    } else {
      commandLength = 0;
      commandOverflow = true;
      Serial.println(F("ERR,line too long"));
    }
  }
}

void setup() {
  pinMode(ENCODER1_A_PIN, INPUT);
  pinMode(ENCODER1_B_PIN, INPUT);
  pinMode(ENCODER2_A_PIN, INPUT);
  pinMode(ENCODER2_B_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), onEncoder1A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), onEncoder2A, RISING);

  Serial.begin(115200);
  printMetadata();
  Serial.println(F("Commands: Z=zero both, Z1/Z2=zero one, P=print, F=timed, L=legacy"));
}

void loop() {
  readCommands();

  static unsigned long lastPrintMs = 0;
  const unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printData();
  }
}
