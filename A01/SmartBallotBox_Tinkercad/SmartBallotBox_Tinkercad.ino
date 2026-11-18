/*
 * ============================================================
 *  Smart Voting Booth - TINKERCAD SIMULATION VERSION (v2)
 *  Built from the hardware sketch (same control logic, same pins).
 * ============================================================
 *
 *  Tinkercad has no RC522 RFID reader and no VL53L1X laser sensor, so:
 *    RFID card  -> type the card number in the Serial Monitor
 *                  (1 = registered voter, 9 = poll worker,
 *                   any other number = unregistered card)
 *    VL53L1X    -> HC-SR04 ultrasonic sensor (TRIG = 9, ECHO = 10)
 *
 *  Everything else is the same as the hardware: I2C LCD (0x27),
 *  green + red LED, button, buzzer, servo, state machine.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

const uint8_t LCD_I2C_ADDR = 0x27;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);

// ------------------------- Pins -----------------------------
const uint8_t PIN_SERVO     = 8;
const uint8_t PIN_US_TRIG   = 9;    // ultrasonic stand-in for the laser sensor
const uint8_t PIN_US_ECHO   = 10;
const uint8_t PIN_LED_GREEN = A0;
const uint8_t PIN_LED_RED   = A1;
const uint8_t PIN_BUTTON    = A2;   // button to GND, internal pull-up
const uint8_t PIN_BUZZER    = A3;
// I2C LCD: SDA = A4, SCL = A5

// ---------------------- Parameters --------------------------
const uint16_t PRESENCE_ON_MM   = 400;
const uint16_t PRESENCE_OFF_MM  = 500;
const uint16_t NO_TARGET_MM     = 8190;
const uint8_t  FILTER_SIZE      = 5;
const unsigned long ECHO_TIMEOUT_US = 30000;

const unsigned long SENSOR_PERIOD_MS  = 100;
const unsigned long ENTER_TIMEOUT_MS  = 15000;
const unsigned long VOTE_TIMEOUT_MS   = 60000;
const unsigned long LEAVE_GRACE_MS    = 3000;
const unsigned long SLOT_OPEN_MS      = 4000;
const unsigned long MESSAGE_MS        = 2500;
const unsigned long DEBOUNCE_MS       = 50;
const unsigned long HEARTBEAT_MS      = 1000;

// Servo: the simulator servo is positional, so the commands are angles
const uint8_t SERVO_OPEN_CMD  = 120;
const uint8_t SERVO_CLOSE_CMD = 60;
const uint8_t SERVO_STOP_CMD  = 90;
const unsigned long SERVO_MOVE_MS = 600;

// ------------------- Voter registry -------------------------
struct Voter {
  long cardId;
  const char* name;
  bool hasVoted;
};

Voter voters[] = {
  { 1, "Voter 1", false },
};
const uint8_t NUM_VOTERS = sizeof(voters) / sizeof(voters[0]);
const long ADMIN_CARD = 9;

Servo slotServo;

// ------------------------ State -----------------------------
enum State { ST_IDLE, ST_MESSAGE, ST_WAIT_ENTER, ST_IN_BOOTH, ST_SLOT_OPEN };
enum SlotPhase { SLOT_OPENING, SLOT_HOLD, SLOT_CLOSING };
SlotPhase slotPhase = SLOT_OPENING;
State state = ST_IDLE;
const char* const STATE_NAMES[] = { "IDLE", "MESSAGE", "WAIT_ENTER", "IN_BOOTH", "SLOT_OPEN" };

unsigned long stateStartMs = 0;
bool pollOpen      = true;
int  currentVoter  = -1;
uint16_t voteCount = 0;

uint16_t distBuf[FILTER_SIZE];
uint8_t  distIdx = 0;
uint16_t distanceMm = NO_TARGET_MM;
bool     voterPresent = false;
unsigned long lastSensorMs = 0;
unsigned long absentSinceMs = 0;

bool lastButtonReading = HIGH;
bool buttonState       = HIGH;
unsigned long lastDebounceMs = 0;

unsigned long lastHeartbeatMs = 0;
bool heartbeatOn = false;
int  lastShownSec = -1;

// ------------------ Function prototypes ---------------------
void enterState(State newState);
void showError(const char* l1, const char* l2);
void showInfo(const char* l1, const char* l2);
void handleCard();
uint16_t readDistanceMm();
void updateDistance();
void openSlot();
void updateSlot(unsigned long elapsed);
void servoRun(uint8_t cmd);
void servoStop();
void printStats();
bool buttonPressed();
void setLeds(bool green, bool red);
void heartbeat();
void showCountdown(unsigned long remainingMs);
void lcdShow(const char* line1, const char* line2);
void beep(unsigned int freq, unsigned long ms);

// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("=== Smart Voting Booth (Tinkercad) - starting ==="));
  Serial.println(F("Type a card number and press Send: 1 = voter, 9 = poll worker"));

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);

  lcd.init();
  lcd.backlight();
  lcdShow("Smart Ballot", "Starting...");

  servoStop();

  for (uint8_t i = 0; i < FILTER_SIZE; i++) distBuf[i] = NO_TARGET_MM;
  Serial.println(F("[SENSOR] Distance sensor ready"));

  enterState(ST_IDLE);
}

void loop() {
  updateDistance();
  bool pressed = buttonPressed();
  unsigned long elapsed = millis() - stateStartMs;

  switch (state) {
    case ST_IDLE:
      heartbeat();
      handleCard();
      break;

    case ST_MESSAGE:
      if (elapsed >= MESSAGE_MS) enterState(ST_IDLE);
      break;

    case ST_WAIT_ENTER:
      showCountdown(ENTER_TIMEOUT_MS - elapsed);
      if (voterPresent) {
        enterState(ST_IN_BOOTH);
      } else if (elapsed >= ENTER_TIMEOUT_MS) {
        Serial.println(F("[TIMEOUT] Voter did not enter the booth - session cancelled"));
        showError("Timeout", "Scan card again");
      }
      break;

    case ST_IN_BOOTH:
      if (pressed) {
        openSlot();
      } else if (!voterPresent && millis() - absentSinceMs >= LEAVE_GRACE_MS) {
        Serial.println(F("[CANCEL] Voter left the booth before confirming"));
        showError("Voter left", "Vote cancelled");
      } else if (elapsed >= VOTE_TIMEOUT_MS) {
        Serial.println(F("[TIMEOUT] Voting time expired"));
        showError("Time is up", "Vote cancelled");
      }
      break;

    case ST_SLOT_OPEN:
      updateSlot(elapsed);
      break;
  }
}

// ============================================================
void enterState(State newState) {
  Serial.print(F("[STATE] "));
  Serial.print(STATE_NAMES[state]);
  Serial.print(F(" -> "));
  Serial.println(STATE_NAMES[newState]);

  state = newState;
  stateStartMs = millis();
  lastShownSec = -1;

  switch (newState) {
    case ST_IDLE:
      currentVoter = -1;
      setLeds(false, !pollOpen);
      if (pollOpen) lcdShow("Welcome!", "Scan voter card");
      else          lcdShow("Poll is CLOSED", "");
      break;
    case ST_WAIT_ENTER:
      setLeds(true, false);
      lcdShow(voters[currentVoter].name, "Enter booth");
      break;
    case ST_IN_BOOTH:
      setLeds(true, false);
      Serial.print(F("[SENSOR] Voter inside booth, distance = "));
      Serial.print(distanceMm);
      Serial.println(F(" mm"));
      lcdShow("Make your choice", "Press to confirm");
      break;
    case ST_SLOT_OPEN:
      setLeds(true, false);
      lcdShow("Insert envelope", "Thank you!");
      break;
    case ST_MESSAGE:
      break;
  }
}

void showError(const char* l1, const char* l2) {
  lcdShow(l1, l2);
  setLeds(false, true);
  beep(300, 400);
  enterState(ST_MESSAGE);
}

void showInfo(const char* l1, const char* l2) {
  lcdShow(l1, l2);
  enterState(ST_MESSAGE);
}

// ------------- "RFID" via Serial Monitor --------------------
void handleCard() {
  if (Serial.available() == 0) return;
  long card = Serial.parseInt();
  while (Serial.available()) Serial.read();
  if (card == 0) return;

  Serial.print(F("[RFID-SIM] Card scanned: "));
  Serial.println(card);

  if (card == ADMIN_CARD) {
    pollOpen = !pollOpen;
    Serial.print(F("[ADMIN] Poll is now "));
    Serial.println(pollOpen ? F("OPEN") : F("CLOSED"));
    printStats();
    char line2[17];
    snprintf(line2, sizeof(line2), "Votes: %u/%u", voteCount, NUM_VOTERS);
    beep(1500, 100);
    showInfo(pollOpen ? "Poll OPENED" : "Poll CLOSED", line2);
    return;
  }

  if (!pollOpen) {
    Serial.println(F("[DENIED] Poll is closed"));
    showError("Poll is closed", "Come back later");
    return;
  }

  int idx = -1;
  for (uint8_t i = 0; i < NUM_VOTERS; i++) {
    if (voters[i].cardId == card) idx = i;
  }

  if (idx < 0) {
    Serial.println(F("[DENIED] Card not registered"));
    showError("Not registered", "Access denied");
    return;
  }

  if (voters[idx].hasVoted) {
    Serial.print(F("[DENIED] Double vote attempt by "));
    Serial.println(voters[idx].name);
    showError("Already voted!", "Access denied");
    return;
  }

  currentVoter = idx;
  Serial.print(F("[OK] Authorized: "));
  Serial.println(voters[idx].name);
  beep(2000, 100);
  enterState(ST_WAIT_ENTER);
}

// ------------- Ultrasonic stand-in for the laser ------------
uint16_t readDistanceMm() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  unsigned long us = pulseIn(PIN_US_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return NO_TARGET_MM;
  return (uint16_t)(us * 0.343 / 2.0);
}

void updateDistance() {
  if (millis() - lastSensorMs < SENSOR_PERIOD_MS) return;
  lastSensorMs = millis();

  distBuf[distIdx] = readDistanceMm();
  distIdx = (distIdx + 1) % FILTER_SIZE;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < FILTER_SIZE; i++) sum += distBuf[i];
  distanceMm = sum / FILTER_SIZE;

  bool wasPresent = voterPresent;
  if (!voterPresent && distanceMm < PRESENCE_ON_MM)  voterPresent = true;
  if (voterPresent  && distanceMm > PRESENCE_OFF_MM) voterPresent = false;

  if (wasPresent != voterPresent) {
    Serial.print(F("[SENSOR] Presence "));
    Serial.print(voterPresent ? F("DETECTED") : F("LOST"));
    Serial.print(F(" at "));
    Serial.print(distanceMm);
    Serial.println(F(" mm"));
    if (!voterPresent) absentSinceMs = millis();
  }
}

// ------------------------------------------------------------
void openSlot() {
  voters[currentVoter].hasVoted = true;
  voteCount++;
  slotPhase = SLOT_OPENING;
  servoRun(SERVO_OPEN_CMD);
  Serial.print(F("[VOTE] Vote #"));
  Serial.print(voteCount);
  Serial.print(F(" confirmed by "));
  Serial.println(voters[currentVoter].name);
  Serial.println(F("[SERVO] Opening ballot slot..."));
  beep(2500, 150);
  enterState(ST_SLOT_OPEN);
}

void updateSlot(unsigned long elapsed) {
  if (slotPhase == SLOT_OPENING && elapsed >= SERVO_MOVE_MS) {
    servoStop();
    slotPhase = SLOT_HOLD;
    Serial.println(F("[SERVO] Ballot slot open"));
  } else if (slotPhase == SLOT_HOLD && elapsed >= SERVO_MOVE_MS + SLOT_OPEN_MS) {
    servoRun(SERVO_CLOSE_CMD);
    slotPhase = SLOT_CLOSING;
    Serial.println(F("[SERVO] Closing ballot slot..."));
  } else if (slotPhase == SLOT_CLOSING && elapsed >= 2 * SERVO_MOVE_MS + SLOT_OPEN_MS) {
    servoStop();
    Serial.println(F("[SERVO] Ballot slot closed"));
    printStats();
    enterState(ST_IDLE);
  }
}

void servoRun(uint8_t cmd) {
  if (!slotServo.attached()) slotServo.attach(PIN_SERVO);
  slotServo.write(cmd);
}

void servoStop() {
  if (slotServo.attached()) {
    slotServo.detach();
  }
}

void printStats() {
  Serial.print(F("[STATS] Votes cast: "));
  Serial.print(voteCount);
  Serial.print(F(" / "));
  Serial.print(NUM_VOTERS);
  Serial.print(F(" registered ("));
  Serial.print(100.0 * voteCount / NUM_VOTERS, 0);
  Serial.println(F("% turnout)"));
}

bool buttonPressed() {
  bool reading = digitalRead(PIN_BUTTON);
  if (reading != lastButtonReading) lastDebounceMs = millis();
  lastButtonReading = reading;

  if (millis() - lastDebounceMs > DEBOUNCE_MS && reading != buttonState) {
    buttonState = reading;
    if (buttonState == LOW) {
      Serial.println(F("[BUTTON] Pressed"));
      return true;
    }
  }
  return false;
}

void setLeds(bool green, bool red) {
  digitalWrite(PIN_LED_GREEN, green ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,   red   ? HIGH : LOW);
}

void heartbeat() {
  if (!pollOpen) return;
  if (millis() - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = millis();
    heartbeatOn = !heartbeatOn;
    digitalWrite(PIN_LED_GREEN, heartbeatOn ? HIGH : LOW);
  }
}

void showCountdown(unsigned long remainingMs) {
  int sec = remainingMs / 1000 + 1;
  if (sec == lastShownSec) return;
  lastShownSec = sec;
  char line2[17];
  snprintf(line2, sizeof(line2), "Enter booth %2ds", sec);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void lcdShow(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void beep(unsigned int freq, unsigned long ms) {
  tone(PIN_BUZZER, freq, ms);
}
