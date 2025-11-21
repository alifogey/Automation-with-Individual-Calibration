/*
 * RAMPS 1.4 Pump Controller for QML/Python backend
 * Commands (newline-terminated JSON at 115200 baud):
 *
 *   // Prime and constant flow:
 *   {"prime": N}
 *   {"pump": N, "flow": F}                 // constant flow in µL/min
 *   {"stop": N}
 *   {"stop_all": true}
 *
 *   // Pulsatile waveform (per pump):
 *   {"wave":{
 *       "pump":     N,
 *       "shape":    "Square" | "Sinusoidal" | "off",
 *       "period":   T_seconds,
 *       "duty":     D_percent,        // only used for Square
 *       "flow":     F_base,           // fallback if min/max not set
 *       "min_flow": F_min,           // optional (µL/min)
 *       "max_flow": F_max            // optional (µL/min)
 *   }}
 *
 *   - Square: flow toggles between min_flow and max_flow
 *   - Sinusoidal: flow varies smoothly between min_flow and max_flow
 */

#include <AccelStepper.h>
#include <math.h>

// ---------- RAMPS 1.4 pins ----------
#define X_EN   38
#define X_DIR   1
#define X_STP   0

#define Y_EN    2
#define Y_DIR   7
#define Y_STP   6

#define Z_EN    8
#define Z_DIR  48
#define Z_STP  46

#define E0_EN  24
#define E0_DIR 28
#define E0_STP 26

#define E1_EN  30
#define E1_DIR 34
#define E1_STP 36

// ---------- Config ----------
#define BAUD 115200

// Map "UI pump number" -> which driver to use.
// Right now: UI Pump 1 -> E1, 2 -> E0, 3 -> X, 4 -> Y, 5 -> Z.
enum DriverId {DRV_E1 = 0, DRV_E0, DRV_X, DRV_Y, DRV_Z, DRV_COUNT};
const DriverId PUMP_MAP[] = {DRV_E1, DRV_E0, DRV_X, DRV_Y, DRV_Z};

// Prime / flow mapping
const float PRIME_SPS           = 250.0f;   // priming speed (steps/s)
const float FLOW_UL_PER_MIN_MAX = 60.0f;    // µL/min that maps to MAX_SPS
const float MAX_SPS             = 720.0f;   // steps/s at that max (tune if needed)

// ---------- Objects ----------
AccelStepper steppers[DRV_COUNT] = {
  AccelStepper(AccelStepper::DRIVER, E1_STP, E1_DIR),  // DRV_E1
  AccelStepper(AccelStepper::DRIVER, E0_STP, E0_DIR),  // DRV_E0
  AccelStepper(AccelStepper::DRIVER, X_STP,  X_DIR ),  // DRV_X
  AccelStepper(AccelStepper::DRIVER, Y_STP,  Y_DIR ),  // DRV_Y
  AccelStepper(AccelStepper::DRIVER, Z_STP,  Z_DIR )   // DRV_Z
};

const uint8_t EN_PINS[DRV_COUNT] = { E1_EN, E0_EN, X_EN, Y_EN, Z_EN };

// ---------- State ----------
float    targetSps[DRV_COUNT]    = {0,0,0,0,0};
uint32_t primeUntilMs[DRV_COUNT] = {0,0,0,0,0};   // kept for completeness
String   line;

// Pulsatile wave state per driver
struct WaveState {
  bool     active;      // true = use pulsatile control
  bool     isSquare;    // true = square, false = sinusoidal
  float    minFlow;     // µL/min (low)
  float    maxFlow;     // µL/min (high)
  float    periodSec;   // seconds
  float    duty;        // 0..1 (only for square)
  uint32_t startMs;     // millis at which wave was started
};

WaveState waves[DRV_COUNT];

// ---------- Helpers ----------
void enableDriver(DriverId d, bool en) {
  // RAMPS enable is active LOW
  digitalWrite(EN_PINS[d], en ? LOW : HIGH);
}

float flowToSps(float flowUlPerMin) {
  if (flowUlPerMin <= 0.0f) return 0.0f;
  // clamp to avoid insane speeds
  if (flowUlPerMin > FLOW_UL_PER_MIN_MAX)
    flowUlPerMin = FLOW_UL_PER_MIN_MAX;
  return (flowUlPerMin / FLOW_UL_PER_MIN_MAX) * MAX_SPS;
}

void setSpeedSps(DriverId d, float sps) {
  if (sps < 0) sps = -sps;
  targetSps[d] = sps;

  if (sps > 0.0f) {
    enableDriver(d, true);
    steppers[d].setMaxSpeed(sps);
    steppers[d].setSpeed(sps);  // constant speed; runSpeed() in loop
  } else {
    steppers[d].setSpeed(0);
    enableDriver(d, false);
  }
}

void stopAll() {
  for (int i = 0; i < DRV_COUNT; ++i) {
    targetSps[i]    = 0;
    primeUntilMs[i] = 0;
    waves[i].active = false;
    steppers[i].setSpeed(0);
    enableDriver((DriverId)i, false);
  }
}

static int readIntAfter(const String& s, const String& key, int deflt=-1) {
  int k = s.indexOf(key);
  if (k < 0) return deflt;
  k = s.indexOf(':', k);
  if (k < 0) return deflt;
  int j = k + 1;
  while (j < (int)s.length() && s[j] == ' ') j++;
  return s.substring(j).toInt();
}

static float readFloatAfter(const String& s, const String& key, float deflt=0.0f) {
  int k = s.indexOf(key);
  if (k < 0) return deflt;
  k = s.indexOf(':', k);
  if (k < 0) return deflt;
  int j = k + 1;
  while (j < (int)s.length() && s[j] == ' ') j++;
  return s.substring(j).toFloat();
}

// Read a quoted string value after a key, e.g. "shape": "Sinusoidal"
static String readStringAfter(const String& s, const String& key, const String& deflt="") {
  int k = s.indexOf(key);
  if (k < 0) return deflt;
  k = s.indexOf(':', k);
  if (k < 0) return deflt;
  int j = k + 1;
  while (j < (int)s.length() && s[j] == ' ') j++;
  if (j >= (int)s.length()) return deflt;

  if (s[j] == '"') {
    int m = s.indexOf('"', j+1);
    if (m < 0) return deflt;
    return s.substring(j+1, m);
  }
  int m = j;
  while (m < (int)s.length() && s[m] != ',' && s[m] != '}' && s[m] != ']') m++;
  return s.substring(j, m);
}

// ---------- Command handler ----------
void handleLine(const String& s) {
  // -------- PRIME (continuous, until stopped) --------
  if (s.indexOf("\"prime\"") >= 0) {
    int p = readIntAfter(s, "\"prime\"");
    if (p >= 1) {
      DriverId d = PUMP_MAP[(p-1) % (sizeof(PUMP_MAP)/sizeof(PUMP_MAP[0]))];

      // disable any wave on this pump
      waves[d].active = false;

      // run at prime speed continuously (UI must send stop to end)
      setSpeedSps(d, PRIME_SPS);
      primeUntilMs[d] = 0;
      Serial.println(F("{\"ack\":\"prime_on\"}"));
      return;
    }
  }

  // -------- CONSTANT FLOW --------
  // {"pump": N, "flow": F}
  if (s.indexOf("\"pump\"") >= 0 && s.indexOf("\"flow\"") >= 0 && s.indexOf("\"wave\"") < 0) {
    int p = readIntAfter(s, "\"pump\"");
    float flow = readFloatAfter(s, "\"flow\"");
    if (p >= 1) {
      DriverId d = PUMP_MAP[(p-1) % (sizeof(PUMP_MAP)/sizeof(PUMP_MAP[0]))];

      // override any wave: constant wins
      waves[d].active = false;

      float sps = flowToSps(flow);
      setSpeedSps(d, sps);
      primeUntilMs[d] = 0;
      Serial.println(F("{\"ack\":\"set_flow\"}"));
      return;
    }
  }

  // -------- PULSATILE WAVE CONTROL --------
  // {"wave":{"pump":N,"shape":"Square"/"Sinusoidal"/"off","period":T,"duty":D,"flow":F,"min_flow":Fmin,"max_flow":Fmax}}
  if (s.indexOf("\"wave\"") >= 0) {
    int p = readIntAfter(s, "\"pump\"");
    if (p >= 1) {
      DriverId d = PUMP_MAP[(p-1) % (sizeof(PUMP_MAP)/sizeof(PUMP_MAP[0]))];

      String shape = readStringAfter(s, "\"shape\"");
      float baseFlow = readFloatAfter(s, "\"flow\"");
      float period   = readFloatAfter(s, "\"period\"");
      float dutyPct  = readFloatAfter(s, "\"duty\"");
      float minF     = readFloatAfter(s, "\"min_flow\"");
      float maxF     = readFloatAfter(s, "\"max_flow\"");

      // normalize
      if (period <= 0.0f) period = 1.0f;

      // default min/max if not provided
      bool minValid = (minF > 0.0f);
      bool maxValid = (maxF > 0.0f);

      if (!minValid && !maxValid) {
        // fall back to 0..baseFlow
        minF = 0.0f;
        maxF = (baseFlow > 0.0f) ? baseFlow : 0.0f;
      } else if (!minValid && maxValid) {
        minF = 0.0f;
      } else if (minValid && !maxValid) {
        maxF = (baseFlow > minF) ? baseFlow : minF;
      }

      // ensure ordering
      if (maxF < minF) {
        float tmp = maxF;
        maxF = minF;
        minF = tmp;
      }

      // turn OFF pulsatile if requested or if we ended up with no range
      String shapeLower = shape;
      shapeLower.toLowerCase();
      if (shapeLower == "off" || maxF <= 0.0f) {
        waves[d].active = false;
        float sps = flowToSps(baseFlow);
        setSpeedSps(d, sps);   // possibly 0 if flow<=0
        Serial.println(F("{\"ack\":\"wave_off\"}"));
        return;
      }

      // configure wave
      waves[d].active    = true;
      waves[d].minFlow   = minF;
      waves[d].maxFlow   = maxF;
      waves[d].periodSec = period;
      waves[d].startMs   = millis();

      // shape
      shapeLower = shape;
      shapeLower.toLowerCase();
      if (shapeLower.startsWith("square")) {
        waves[d].isSquare = true;
        float duty = dutyPct / 100.0f;
        if (duty <= 0.0f) duty = 0.5f;
        if (duty >= 1.0f) duty = 0.99f;
        waves[d].duty = duty;
      } else {
        // default to sinusoidal
        waves[d].isSquare = false;
        waves[d].duty     = 0.5f;  // unused but sane
      }

      Serial.println(F("{\"ack\":\"wave\"}"));
      return;
    }
  }

  // -------- STOP ALL --------
  if (s.indexOf("\"stop_all\"") >= 0) {
    stopAll();
    Serial.println(F("{\"ack\":\"stop_all\"}"));
    return;
  }

  // -------- STOP SINGLE PUMP --------
  if (s.indexOf("\"stop\"") >= 0) {
    int p = readIntAfter(s, "\"stop\"");
    if (p >= 1) {
      DriverId d = PUMP_MAP[(p-1) % (sizeof(PUMP_MAP)/sizeof(PUMP_MAP[0]))];
      setSpeedSps(d, 0.0f);
      primeUntilMs[d] = 0;
      waves[d].active = false;
      Serial.println(F("{\"ack\":\"stop\"}"));
      return;
    }
  }
}

// ---------- Setup & loop ----------
void setup() {
  Serial.begin(BAUD);

  for (int i = 0; i < DRV_COUNT; ++i) {
    pinMode(EN_PINS[i], OUTPUT);
    enableDriver((DriverId)i, false);  // disabled at boot
    steppers[i].setAcceleration(2000);
    steppers[i].setMaxSpeed(1500);
    steppers[i].setSpeed(0);

    waves[i].active    = false;
    waves[i].isSquare  = false;
    waves[i].minFlow   = 0.0f;
    waves[i].maxFlow   = 0.0f;
    waves[i].periodSec = 1.0f;
    waves[i].duty      = 0.5f;
    waves[i].startMs   = millis();
  }

  Serial.println(F("{\"status\":\"ready\"}"));
}

void loop() {
  // ----- read serial line -----
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) {
        handleLine(line);
        line = "";
      }
    } else {
      line += c;
      if (line.length() > 200) line = ""; // safety reset
    }
  }

  // ----- run motors -----
  uint32_t now = millis();

  for (int i = 0; i < DRV_COUNT; ++i) {
    DriverId d = (DriverId)i;

    if (waves[i].active) {
      float period = (waves[i].periodSec <= 0.0f) ? 1.0f : waves[i].periodSec;
      float tSec   = (now - waves[i].startMs) / 1000.0f;
      float phase  = fmod(tSec, period) / period;  // 0..1
      float flow   = 0.0f;

      if (waves[i].isSquare) {
        float duty = waves[i].duty;
        if (duty <= 0.0f) duty = 0.5f;
        if (duty >= 1.0f) duty = 0.99f;
        // ON during [0, duty) at max, OFF at min
        if (phase < duty)
          flow = waves[i].maxFlow;
        else
          flow = waves[i].minFlow;
      } else {
        // sinusoidal between minFlow and maxFlow
        float s = sinf(2.0f * 3.14159265f * phase); // -1..1
        float factor = 0.5f + 0.5f * s;             // 0..1
        flow = waves[i].minFlow +
               factor * (waves[i].maxFlow - waves[i].minFlow);
      }

      float sps = flowToSps(flow);
      setSpeedSps(d, sps);
    }

    // keep steppers moving at the commanded speed
    steppers[i].runSpeed();
  }
}
