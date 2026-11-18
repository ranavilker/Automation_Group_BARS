/*
 * ============================================================
 *  Smart Voting Booth (עמדת הצבעה חכמה) – Arduino prototype
 *  BGU – Automation & Integrated Systems – Practical Exercise A
 * ============================================================
 *
 *  Control task:
 *    1. Voter scans an RFID card (RC522).
 *    2. System checks: is the poll open? is the card registered?
 *       has this voter already voted?
 *    3. If authorized – the laser ToF sensor (VL53L1X) must detect
 *       the voter standing inside the booth (behind the screen).
 *    4. Inside the booth the voter presses the button to confirm.
 *    5. The servo opens the ballot-box slot for a few seconds,
 *       the vote is counted and the voter is marked as "voted".
 *    6. A poll-worker (admin) card shows the vote count and
 *       toggles the poll OPEN / CLOSED.
 *
 *  Hardware: Arduino UNO, 16x2 LCD, green + red LED, push button,
 *            RC522 RFID (3.3V!), TOF400C-VL53L1X laser range sensor,
 *            MG995 360-degree (continuous rotation) servo, optional buzzer.
 *
 *  Libraries (Arduino IDE -> Tools -> Manage Libraries):
 *    - "MFRC522"  by GithubCommunity (miguelbalboa)
 *    - "VL53L1X"  by Pololu
 *    - "LiquidCrystal I2C" by Frank de Brabander (only if USE_I2C_LCD = 1)
 *    - Servo, SPI, Wire, LiquidCrystal – built in
 */

#include <SPI.h>
#include <Wire.h>
#include <MFRC522.h>
#include <VL53L1X.h>
#include <Servo.h>

// ------------------------------------------------------------
//  LCD type: 1 = LCD with I2C backpack (4 wires),
//            0 = plain 16x2 LCD (parallel, 6 data wires + pot)
// ------------------------------------------------------------
#define USE_I2C_LCD 1

#if USE_I2C_LCD
  #include <LiquidCrystal_I2C.h>
  const uint8_t LCD_I2C_ADDR = 0x27;          // if the screen stays blank try 0x3F
  LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);
#else
  #include <LiquidCrystal.h>
  //               RS  E  D4 D5 D6 D7
  LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
#endif

// Set to 1 to print every distance reading (used for the sensitivity tests)
#define DEBUG_DISTANCE 0

// ------------------------- Pins -----------------------------
const uint8_t PIN_SERVO     = 8;
const uint8_t PIN_RFID_RST  = 9;
const uint8_t PIN_RFID_SS   = 10;   // SPI: MOSI=11, MISO=12, SCK=13
const uint8_t PIN_LED_GREEN = A0;
const uint8_t PIN_LED_RED   = A1;
const uint8_t PIN_BUTTON    = A2;   // button to GND, internal pull-up
const uint8_t PIN_BUZZER    = A3;   // optional
// VL53L1X (and I2C LCD) use the I2C bus: SDA=A4, SCL=A5
// VL53L1X INT and SHUT pins are left unconnected

// ---------------------- Parameters --------------------------
const uint16_t PRESENCE_ON_MM   = 400;   // voter "inside" when closer than this
const uint16_t PRESENCE_OFF_MM  = 500;   // voter "left" when farther (hysteresis)
const uint16_t MAX_RANGE_MM     = 4000;  // VL53L1X absolute max (datasheet); anything above = invalid
const uint16_t NO_TARGET_MM     = 8190;  // value used for invalid readings
const uint8_t  FILTER_SIZE      = 5;     // moving-average window

const unsigned long SENSOR_PERIOD_MS  = 100;    // distance sampling period
const unsigned long ENTER_TIMEOUT_MS  = 15000;  // time to enter the booth after scan
const unsigned long VOTE_TIMEOUT_MS   = 60000;  // max time inside the booth
const unsigned long LEAVE_GRACE_MS    = 3000;   // voter may step out briefly
const unsigned long SLOT_OPEN_MS      = 4000;   // ballot slot open time
const unsigned long MESSAGE_MS        = 2500;   // info / error message time
const unsigned long DEBOUNCE_MS       = 50;
const unsigned long HEARTBEAT_MS      = 1000;   // green LED blink in idle

// --- Continuous-rotation (360 deg) servo ---
// write() sets SPEED and DIRECTION, not angle: 90 = stop,
// below 90 = one direction, above 90 = the other direction.
// The slot flap is opened by rotating for SERVO_MOVE_MS and closed
// by rotating back for the same time. Calibrate these 3 values:
const uint8_t SERVO_OPEN_CMD  = 120;    // swap with SERVO_CLOSE_CMD if the flap moves the wrong way
const uint8_t SERVO_CLOSE_CMD = 55;     // calibrated: slightly slower close so the flap does not creep
const uint8_t SERVO_STOP_CMD  = 90;
const unsigned long SERVO_MOVE_MS = 600;  // rotation time for ~a quarter turn (calibrated)

// ------------------- Voter registry -------------------------
// Scan your cards once – unknown UIDs are printed to the Serial
// Monitor – then copy them here.
struct Voter {
  byte uid[4];
  const char* name;
  bool hasVoted;
};

Voter voters[] = {
  { {0x43, 0x3A, 0xDD, 0xA9}, "Voter 1", false },   // white card
};
const uint8_t NUM_VOTERS = sizeof(voters) / sizeof(voters[0]);

const byte ADMIN_UID[4] = {0x2A, 0xBB, 0x14, 0xB1};  // blue key tag = poll worker

// ------------------------ Objects ---------------------------
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
VL53L1X tof;
Servo   slotServo;

// ------------------------ State -----------------------------
enum State { ST_IDLE, ST_MESSAGE, ST_WAIT_ENTER, ST_IN_BOOTH, ST_SLOT_OPEN };
enum SlotPhase { SLOT_OPENING, SLOT_HOLD, SLOT_CLOSING };
SlotPhase slotPhase = SLOT_OPENING;
State state = ST_IDLE;
const char* const STATE_NAMES[] = { "IDLE", "MESSAGE", "WAIT_ENTER", "IN_BOOTH", "SLOT_OPEN" };

unsigned long stateStartMs = 0;
bool pollOpen      = true;
int  currentVoter  = -1;      // index in voters[]
uint16_t voteCount = 0;

// distance filter
uint16_t distBuf[FILTER_SIZE];
uint8_t  distIdx = 0;
uint16_t distanceMm = NO_TARGET_MM;   // filtered value
bool     voterPresent = false;
unsigned long lastSensorMs = 0;
unsigned long absentSinceMs = 0;

// button
bool lastButtonReading = HIGH;
bool buttonState       = HIGH;
unsigned long lastDebounceMs = 0;

// misc
unsigned long lastHeartbeatMs = 0;
bool heartbeatOn = false;
int  lastShownSec = -1;

// ------------------ Function prototypes ---------------------
void enterState(State newState);
void showError(const char* l1, const char* l2);
void showInfo(const char* l1, const char* l2);
void handleCard();
int  findVoter();
bool uidEquals(const byte* ref);
void printUid(const byte* uid, byte size);
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
void initLcd();
void lcdShow(const char* line1, const char* line2);
void beep(unsigned int freq, unsigned long ms);

// ============================================================
//                         SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("=== Smart Voting Booth – starting ==="));

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  initLcd();
  lcdShow("Smart Ballot", "Starting...");

  servoStop();   // servo stays detached (= stopped) until a vote

  SPI.begin();
  rfid.PCD_Init();
  Serial.print(F("[RFID] RC522 firmware version: 0x"));
  Serial.println(rfid.PCD_ReadRegister(MFRC522::VersionReg), HEX);   // 0x91/0x92 = OK

  Wire.begin();
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println(F("[ERROR] VL53L1X not found – check wiring (SDA=A4, SCL=A5)"));
    lcdShow("Sensor error!", "Check VL53L1X");
    digitalWrite(PIN_LED_RED, HIGH);
    while (true) { }   // stop – the system cannot work without the sensor
  }
  tof.setDistanceMode(VL53L1X::Short);         // up to ~1.3 m, best immunity to ambient light
  tof.setMeasurementTimingBudget(50000);       // 50 ms per measurement
  tof.startContinuous(50);                     // new measurement every 50 ms
  for (uint8_t i = 0; i < FILTER_SIZE; i++) distBuf[i] = NO_TARGET_MM;
  Serial.println(F("[SENSOR] VL53L1X ready"));

  enterState(ST_IDLE);
}

// ============================================================
//                          LOOP
// ============================================================
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
        Serial.println(F("[TIMEOUT] Voter did not enter the booth – session cancelled"));
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
//                    STATE MANAGEMENT
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
      // text is written by the caller
      break;
  }
}

// Shows an error for MESSAGE_MS, then returns to idle
void showError(const char* l1, const char* l2) {
  lcdShow(l1, l2);
  setLeds(false, true);
  beep(300, 400);
  enterState(ST_MESSAGE);
}

// Shows an info message for MESSAGE_MS, then returns to idle
void showInfo(const char* l1, const char* l2) {
  lcdShow(l1, l2);
  enterState(ST_MESSAGE);
}

// ============================================================
//                          RFID
// ============================================================
void handleCard() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  Serial.print(F("[RFID] Card scanned, UID:"));
  printUid(rfid.uid.uidByte, rfid.uid.size);

  bool isAdmin   = uidEquals(ADMIN_UID);
  int  voterIdx  = findVoter();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  if (isAdmin) {
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

  if (voterIdx < 0) {
    Serial.println(F("[DENIED] Card not registered (copy the UID above into voters[] to register it)"));
    showError("Not registered", "Access denied");
    return;
  }

  if (voters[voterIdx].hasVoted) {
    Serial.print(F("[DENIED] Double vote attempt by "));
    Serial.println(voters[voterIdx].name);
    showError("Already voted!", "Access denied");
    return;
  }

  currentVoter = voterIdx;
  Serial.print(F("[OK] Authorized: "));
  Serial.println(voters[voterIdx].name);
  beep(2000, 100);
  enterState(ST_WAIT_ENTER);
}

int findVoter() {
  for (uint8_t i = 0; i < NUM_VOTERS; i++) {
    if (uidEquals(voters[i].uid)) return i;
  }
  return -1;
}

bool uidEquals(const byte* ref) {
  if (rfid.uid.size != 4) return false;   // classic MIFARE cards have 4-byte UIDs
  for (uint8_t i = 0; i < 4; i++) {
    if (rfid.uid.uidByte[i] != ref[i]) return false;
  }
  return true;
}

void printUid(const byte* uid, byte size) {
  for (byte i = 0; i < size; i++) {
    Serial.print(F(" 0x"));
    if (uid[i] < 0x10) Serial.print('0');
    Serial.print(uid[i], HEX);
  }
  Serial.println();
}

// ============================================================
//                 LASER DISTANCE SENSOR
// ============================================================
// Samples the VL53L1X every SENSOR_PERIOD_MS, applies a moving
// average and decides presence with hysteresis.
void updateDistance() {
  if (millis() - lastSensorMs < SENSOR_PERIOD_MS) return;
  lastSensorMs = millis();

  uint16_t raw = tof.read();
  // invalid reading (timeout / no target / signal or sigma fail) = "nobody there"
  if (tof.timeoutOccurred() ||
      tof.ranging_data.range_status != VL53L1X::RangeValid ||
      raw == 0 || raw > MAX_RANGE_MM) {
    raw = NO_TARGET_MM;
  }

  distBuf[distIdx] = raw;
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

#if DEBUG_DISTANCE
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print(F("raw_mm=")); Serial.print(raw);
    Serial.print(F("\tfiltered_mm=")); Serial.println(distanceMm);
  }
#endif
}

// ============================================================
//                    ACTUATORS & UI
// ============================================================
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

// Opening -> hold open -> closing, timed (continuous-rotation servo)
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

// Start rotating (speed/direction command for a 360-degree servo)
void servoRun(uint8_t cmd) {
  if (!slotServo.attached()) slotServo.attach(PIN_SERVO);
  slotServo.write(cmd);
}

// Stop: send the stop command, then detach so no pulses are sent at all.
// A detached continuous servo stays still even if 90 is not its exact neutral.
void servoStop() {
  if (slotServo.attached()) {
    slotServo.write(SERVO_STOP_CMD);
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

// Returns true once per press (falling edge, debounced)
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

// Slow green blink while waiting for a voter (poll open only)
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
  if (sec == lastShownSec) return;   // update the LCD only when the number changes
  lastShownSec = sec;
  char line2[17];
  snprintf(line2, sizeof(line2), "Enter booth %2ds", sec);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void initLcd() {
#if USE_I2C_LCD
  lcd.init();
  lcd.backlight();
#else
  lcd.begin(16, 2);
#endif
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
