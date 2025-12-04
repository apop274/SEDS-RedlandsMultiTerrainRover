//right stick --> forward = right turn, back = left turn
#include <Arduino.h>

// === RC input pins ===
// FS-R6B CH2 -> D2 (forward/back)
// FS-R6B CH3 -> D3 (turn)
#define CH_FWD_PIN  2
#define CH_TURN_PIN 3

// === DFR0601 motor driver wiring ===
// Driver 1 (front motors)
// Motor 1 = front right
// Motor 2 = front left
const uint8_t A_FR = 7;   const uint8_t P_FR = 5;   const uint8_t B_FR = 8;
const uint8_t A_FL = 12;  const uint8_t P_FL = 6;   const uint8_t B_FL = 13;

// Driver 2 (rear motors)
// Motor 1 = back left
// Motor 2 = back right
const uint8_t A_BL = 4;   const uint8_t P_BL = 9;   const uint8_t B_BL = 11;
const uint8_t A_BR = A0;  const uint8_t P_BR = 10;  const uint8_t B_BR = A1;

// Set to true if a wheel runs backwards relative to others
bool INVERT_FR = false;
bool INVERT_FL = false;
bool INVERT_BL = false;
bool INVERT_BR = false;

// RC timing / safety
const unsigned long RC_READ_TIMEOUT_US = 30000UL;  // pulseIn timeout
const unsigned long FAILSAFE_MS        = 300UL;    // stop if stale
const int           DEAD_BAND_US       = 80;       // bigger deadband

unsigned long lastGoodMs     = 0;
uint8_t       goodFrameCount = 0;
bool          rcLocked       = false;

struct Motor {
  uint8_t A;
  uint8_t B;
  uint8_t P;
  bool invert;
};

Motor M_FR = {A_FR, B_FR, P_FR, INVERT_FR};  // front right  (RIGHT side)
Motor M_FL = {A_FL, B_FL, P_FL, INVERT_FL};  // front left   (LEFT side)
Motor M_BL = {A_BL, B_BL, P_BL, INVERT_BL};  // back left    (LEFT side)
Motor M_BR = {A_BR, B_BR, P_BR, INVERT_BR};  // back right   (RIGHT side)

// === Helpers ===
inline void stopMotor(const Motor &m) {
  digitalWrite(m.A, LOW);
  digitalWrite(m.B, LOW);   // L/L = brake/stop on DFR0601
  analogWrite(m.P, 0);
}

inline void driveSigned(const Motor &m, int16_t val) {
  if (val == 0) {
    stopMotor(m);
    return;
  }
  bool forward = (val > 0);
  uint8_t duty = constrain(abs(val), 0, 255);

  bool fwd = forward ^ m.invert;
  // DFR0601: L/H forward, H/L reverse
  digitalWrite(m.A, fwd ? LOW  : HIGH);
  digitalWrite(m.B, fwd ? HIGH : LOW);
  analogWrite(m.P, duty);
}

inline void stopAll() {
  stopMotor(M_FR);
  stopMotor(M_FL);
  stopMotor(M_BL);
  stopMotor(M_BR);
}

// Read one RC channel (µs). Returns 0 on timeout.
unsigned long readRcUs(uint8_t pin) {
  return pulseIn(pin, HIGH, RC_READ_TIMEOUT_US);
}

// Map 1000–2000us to -1.0 .. +1.0 with deadband around 1500
float rcUsToNorm(long us) {
  if (us < 900 || us > 2100) return 0.0f;

  long delta = us - 1500;
  if (abs(delta) <= DEAD_BAND_US) return 0.0f;

  float val = (float)delta / 500.0f;  // about ±500us → ±1.0
  val = constrain(val, -1.0f, 1.0f);
  return val;
}

void setup() {
  // RC inputs
  pinMode(CH_FWD_PIN, INPUT);
  pinMode(CH_TURN_PIN, INPUT);

  // Motor pins
  pinMode(A_FR, OUTPUT); pinMode(B_FR, OUTPUT); pinMode(P_FR, OUTPUT);
  pinMode(A_FL, OUTPUT); pinMode(B_FL, OUTPUT); pinMode(P_FL, OUTPUT);
  pinMode(A_BL, OUTPUT); pinMode(B_BL, OUTPUT); pinMode(P_BL, OUTPUT);
  pinMode(A_BR, OUTPUT); pinMode(B_BR, OUTPUT); pinMode(P_BR, OUTPUT);

  stopAll();  // ensure everything is off

  Serial.begin(115200);
  Serial.println(F("UNO diff drive: CH2=fwd/back, CH3=turn. Front/rear wiring."));
  lastGoodMs     = millis();
  goodFrameCount = 0;
  rcLocked       = false;
}

void loop() {
  // Read RC
  unsigned long usFwd  = readRcUs(CH_FWD_PIN);
  unsigned long usTurn = readRcUs(CH_TURN_PIN);

  bool valid = (usFwd >= 900 && usFwd <= 2100 &&
                usTurn >= 900 && usTurn <= 2100);

  if (valid) {
    lastGoodMs = millis();

    // Build RC lock: require 10 consecutive good frames
    if (!rcLocked) {
      if (goodFrameCount < 10) {
        goodFrameCount++;
        // keep everything stopped while locking
        stopAll();
        return;
      } else {
        rcLocked = true;
        Serial.println(F("RC LOCKED – enabling drive."));
      }
    }

    float fwd  = rcUsToNorm((long)usFwd);   // -1..+1
    float turn = rcUsToNorm((long)usTurn);  // -1..+1

    // Differential mixing
    float left  = fwd + turn;
    float right = fwd - turn;
    left  = constrain(left,  -1.0f, 1.0f);
    right = constrain(right, -1.0f, 1.0f);

    int16_t leftCmd  = (int16_t)(left  * 255.0f);
    int16_t rightCmd = (int16_t)(right * 255.0f);

    // small deadzone
    if (abs(leftCmd)  < 10) leftCmd  = 0;
    if (abs(rightCmd) < 10) rightCmd = 0;

    // LEFT side motors: front left + back left
    driveSigned(M_FL, leftCmd);  // front left
    driveSigned(M_BL, leftCmd);  // back left

    // RIGHT side motors: front right + back right
    driveSigned(M_FR, rightCmd); // front right
    driveSigned(M_BR, rightCmd); // back right

    // Debug if needed:
    // Serial.print("usFwd="); Serial.print(usFwd);
    // Serial.print(" usTurn="); Serial.print(usTurn);
    // Serial.print(" L="); Serial.print(leftCmd);
    // Serial.print(" R="); Serial.println(rightCmd);
  } else {
    // bad frame -> drop lock and stop
    goodFrameCount = 0;
    rcLocked       = false;
    stopAll();
  }

  // Failsafe: if no RC in a while, stop and drop lock
  if (millis() - lastGoodMs > FAILSAFE_MS) {
    stopAll();
    rcLocked       = false;
    goodFrameCount = 0;
  }
}



/*
#include <Arduino.h>

//RC input pins
// FS-R6B CH2 -> D2 (forward/back)
// FS-R6B CH3 -> D3 (turn)
#define CH_FWD_PIN  2
#define CH_TURN_PIN 3

//DFR0601 motor driver wiring
// Driver 1 (front motors)
// Motor 1 = front right
// Motor 2 = front left
const uint8_t A_FR = 7;   const uint8_t P_FR = 5;   const uint8_t B_FR = 8;
const uint8_t A_FL = 12;  const uint8_t P_FL = 6;   const uint8_t B_FL = 13;

//Driver 2 (rear motors)
//Motor 1 = back left
//Motor 2 = back right
const uint8_t A_BL = 4;   const uint8_t P_BL = 9;   const uint8_t B_BL = 11;
const uint8_t A_BR = A0;  const uint8_t P_BR = 10;  const uint8_t B_BR = A1;

//per wheel inversion
bool INVERT_FR = false;
bool INVERT_FL = false;
bool INVERT_BL = false;
bool INVERT_BR = false;

//axis flip if stick feels backwards
const bool REVERSE_FWD  = false;  // flip forward/back
const bool REVERSE_TURN = false;  // flip left/right

// RC timing
const unsigned long RC_READ_TIMEOUT_US = 30000UL;
const unsigned long FAILSAFE_MS        = 300UL;
const int           DEAD_BAND_US       = 60;  // for center

unsigned long lastGoodMs = 0;

struct Motor {
  uint8_t A;
  uint8_t B;
  uint8_t P;
  bool invert;
};

Motor M_FR = {A_FR, B_FR, P_FR, INVERT_FR};  // front right (RIGHT side)
Motor M_FL = {A_FL, B_FL, P_FL, INVERT_FL};  // front left  (LEFT side)
Motor M_BL = {A_BL, B_BL, P_BL, INVERT_BL};  // back left   (LEFT side)
Motor M_BR = {A_BR, B_BR, P_BR, INVERT_BR};  // back right  (RIGHT side);

//ow-level motor helpers
inline void stopMotor(const Motor &m) {
  digitalWrite(m.A, LOW);
  digitalWrite(m.B, LOW);
  analogWrite(m.P, 0);
}

inline void driveMotor(const Motor &m, int16_t speed) {
  if (speed == 0) {
    stopMotor(m);
    return;
  }
  bool forward = (speed > 0);
  uint8_t duty = constrain(abs(speed), 0, 255);

  bool fwd = forward ^ m.invert;
  // DFR0601 logic: L/H = forward, H/L = reverse
  digitalWrite(m.A, fwd ? LOW  : HIGH);
  digitalWrite(m.B, fwd ? HIGH : LOW);
  analogWrite(m.P, duty);
}

inline void stopAll() {
  stopMotor(M_FR);
  stopMotor(M_FL);
  stopMotor(M_BL);
  stopMotor(M_BR);
}

//Read one RC channel (microseconds). Returns 0 on timeout.
unsigned long readRcUs(uint8_t pin) {
  return pulseIn(pin, HIGH, RC_READ_TIMEOUT_US);
}

//Map 1000–2000microseconds to signed speed -255..+255 with deadband
int16_t rcUsToSpeed(long us) {
  if (us < 900 || us > 2100) return 0;
  long delta = us - 1500;
  if (abs(delta) <= DEAD_BAND_US) return 0;

  //map roughly 1000..2000 → -255..+255
  if (delta > 0) {
    // 1500..2000
    return map(delta, DEAD_BAND_US, 500L, 50, 255);  // small min speed
  } else {
    // 1000..1500
    delta = -delta;
    int16_t s = map(delta, DEAD_BAND_US, 500L, 50, 255);
    return -s;
  }
}

void setup() {
  //RC inputs
  pinMode(CH_FWD_PIN, INPUT);
  pinMode(CH_TURN_PIN, INPUT);

  //Motor pins
  pinMode(A_FR, OUTPUT); pinMode(B_FR, OUTPUT); pinMode(P_FR, OUTPUT);
  pinMode(A_FL, OUTPUT); pinMode(B_FL, OUTPUT); pinMode(P_FL, OUTPUT);
  pinMode(A_BL, OUTPUT); pinMode(B_BL, OUTPUT); pinMode(P_BL, OUTPUT);
  pinMode(A_BR, OUTPUT); pinMode(B_BR, OUTPUT); pinMode(P_BR, OUTPUT);

  stopAll();

  Serial.begin(115200);
  Serial.println(F("UNO explicit drive: CH2=fwd/back, CH3=turn."));
  lastGoodMs = millis();
}

void loop() {
  unsigned long usFwd  = readRcUs(CH_FWD_PIN);
  unsigned long usTurn = readRcUs(CH_TURN_PIN);

  bool valid = (usFwd >= 900 && usFwd <= 2100 &&
                usTurn >= 900 && usTurn <= 2100);

  if (!valid) {
    //no good RC -> stop
    stopAll();
  } else {
    lastGoodMs = millis();

    //convert pulses to signed speeds
    int16_t fwdSpeed  = rcUsToSpeed((long)usFwd);   // -255..+255
    int16_t turnSpeed = rcUsToSpeed((long)usTurn);  // -255..+255

    if (REVERSE_FWD)  fwdSpeed  = -fwdSpeed;
    if (REVERSE_TURN) turnSpeed = -turnSpeed;

    //decide mode: drive vs turn vs stop
    int16_t leftCmd  = 0;
    int16_t rightCmd = 0;

    int absFwd  = abs(fwdSpeed);
    int absTurn = abs(turnSpeed);

    if (absFwd < 20 && absTurn < 20) {
      //stick near center: full stop
      leftCmd = rightCmd = 0;
    } else if (absFwd >= absTurn) {
      //mostly forward/back: go straight
      leftCmd  = fwdSpeed;
      rightCmd = fwdSpeed;
    } else {
      //mostly turning: spin in place
      if (turnSpeed > 0) {
        // turn right: left forward, right backward
        leftCmd  =  abs(turnSpeed);
        rightCmd = -abs(turnSpeed);
      } else {
        // turn left: left backward, right forward
        leftCmd  = -abs(turnSpeed);
        rightCmd =  abs(turnSpeed);
      }
    }

    //drive LEFT side motors: front left + back left
    driveMotor(M_FL, leftCmd);
    driveMotor(M_BL, leftCmd);

    // Drive RIGHT side motors: front right + back right
    driveMotor(M_FR, rightCmd);
    driveMotor(M_BR, rightCmd);

 
    // Serial.print("usFwd="); Serial.print(usFwd);
    // Serial.print(" usTurn="); Serial.print(usTurn);
    // Serial.print("  fwd="); Serial.print(fwdSpeed);
    // Serial.print("  turn="); Serial.print(turnSpeed);
    // Serial.print("  L="); Serial.print(leftCmd);
    // Serial.print("  R="); Serial.println(rightCmd);
  }

  //failsafe: if no RC in a while, stop
  if (millis() - lastGoodMs > FAILSAFE_MS) {
    stopAll();
  }
}
*/
