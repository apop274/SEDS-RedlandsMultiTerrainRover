#include <Arduino.h>

// Receiver inputs (5V logic)
#define CH_DIR_PIN  2    // FS-R6B CH2 -> D2
#define CH_SPD_PIN  3    // FS-R6B CH3 -> D3

// -------- DFR0601 wiring --------
// Driver #1 (same as before)
const uint8_t A_M1 = 7;   const uint8_t P_M1 = 5;   const uint8_t B_M1 = 8;
const uint8_t A_M2 = 12;  const uint8_t P_M2 = 6;   const uint8_t B_M2 = 13;

// Driver #2 (remapped off analog pins)
const uint8_t A_M3 = 22;  const uint8_t P_M3 = 9;   const uint8_t B_M3 = 24;
const uint8_t A_M4 = 26;  const uint8_t P_M4 = 10;  const uint8_t B_M4 = 28;

// Invert any wheel if needed
bool INVERT_M1=false, INVERT_M2=false, INVERT_M3=false, INVERT_M4=false;

// RC timing/logic
const unsigned long RC_READ_TIMEOUT_US = 30000UL;   // pulseIn timeout
const unsigned long FAILSAFE_MS        = 300UL;     // stop if stale
const int           DEAD_BAND_US       = 60;        // center deadband

// State
uint8_t speedDuty = 0;
unsigned long lastGoodMs = 0;

struct Motor { uint8_t A,B,P; bool invert; };
Motor motors[4] = {
  {A_M1,B_M1,P_M1,INVERT_M1},
  {A_M2,B_M2,P_M2,INVERT_M2},
  {A_M3,B_M3,P_M3,INVERT_M3},
  {A_M4,B_M4,P_M4,INVERT_M4}
};

inline void stopOne(const Motor& m){
  digitalWrite(m.A, LOW);
  digitalWrite(m.B, LOW);       // L/L = brake on DFR0601
  analogWrite(m.P, 0);
}
inline void driveOne(const Motor& m, int duty, bool forward){
  bool fwd = forward ^ m.invert;
  digitalWrite(m.A, fwd ? LOW  : HIGH);  // L/H forward, H/L reverse
  digitalWrite(m.B, fwd ? HIGH : LOW );
  analogWrite(m.P, constrain(duty, 0, 255));
}
inline void stopAll(){ for (auto &m:motors) stopOne(m); }
inline void fwdAll(uint8_t d){ for (auto &m:motors) driveOne(m, d, true);  }
inline void revAll(uint8_t d){ for (auto &m:motors) driveOne(m, d, false); }

// Read one RC channel (µs). Returns 0 on timeout.
unsigned long readRcUs(uint8_t pin){
  return pulseIn(pin, HIGH, RC_READ_TIMEOUT_US);
}

void setup(){
  // RC inputs
  pinMode(CH_DIR_PIN, INPUT);
  pinMode(CH_SPD_PIN, INPUT);

  // Motor pins
  for (auto &m:motors){
    pinMode(m.A, OUTPUT);
    pinMode(m.B, OUTPUT);
    pinMode(m.P, OUTPUT);    // Mega PWM: use 5,6,9,10 here
  }
  stopAll();

  Serial.begin(115200);
  lastGoodMs = millis();
  Serial.println(F("MEGA RC drive: CH2=dir (D2), CH3=speed (D3). Failsafe 300ms."));
}

void loop(){
  unsigned long usDir = readRcUs(CH_DIR_PIN);  // ~1000..2000
  unsigned long usSpd = readRcUs(CH_SPD_PIN);

  bool valid = (usDir >= 900 && usDir <= 2100 && usSpd >= 900 && usSpd <= 2100);
  if (valid){
    lastGoodMs = millis();

    // Map speed: 1000..2000 -> 0..255
    usSpd = constrain(usSpd, 1000UL, 2000UL);
    speedDuty = map((int)usSpd, 1000, 2000, 0, 255);

    // Direction with deadband
    int dir = 0;
    if      ((int)usDir > 1500 + DEAD_BAND_US) dir = +1;   // forward
    else if ((int)usDir < 1500 - DEAD_BAND_US) dir = -1;   // reverse
    else dir = 0;

    if (dir > 0)      fwdAll(speedDuty);
    else if (dir < 0) revAll(speedDuty);
    else              stopAll();

    // // Debug (optional):
    // Serial.print("CH2="); Serial.print(usDir);
    // Serial.print("  CH3="); Serial.print(usSpd);
    // Serial.print("  duty="); Serial.print(speedDuty);
    // Serial.print("  dir="); Serial.println(dir);
  }

  // Failsafe
  if (millis() - lastGoodMs > FAILSAFE_MS){
    stopAll();
  }
}
