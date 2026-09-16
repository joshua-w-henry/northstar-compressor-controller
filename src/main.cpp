#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>
#include <U8g2lib.h>
#include <Adafruit_FRAM_I2C.h>

// ================================================================
// NorthStar Compressor Controller - Phase 0 Hardware Rebuild
//
// MOSFET outputs, ACTIVE HIGH:
//   D8 = Master MOSFET -> remote 12V automotive relay coil
//   D9 = Start/Stop MOSFET -> remote 12V automotive relay coil
//   D6 = Fault lamp MOSFET
//
// Individual relay-module outputs, ACTIVE LOW:
//   A3 = Unloader solenoid relay
//   A1 = Idle dry-contact relay
//   A2 = Kill dry-contact relay
//
// Inputs:
//   D2 = Pressure switch opto, LOW = CALL, HIGH = FULL
//   D3 = OEM master monitor opto, LOW = master ON
//   D4 = AUTO/OFF opto, LOW = AUTO
//   D5 = Reset/fault clear, LOW = pressed
//   A0 = Battery voltage divider
//   A6 = Force unload analog opto, <600 = active
//   A7 = 200 PSI pressure transducer
//
// CAN RPM:
//   EXT ID 0x0C665500
//   RPM = data[2] << 8 | data[3]
//   Reject instantaneous RPM > 4000
// ================================================================

// ---------------------- Pin map ----------------------

const uint8_t PIN_PRESSURE_SWITCH = 2;
const uint8_t PIN_MASTER_MONITOR  = 3;
const uint8_t PIN_AUTO_SWITCH     = 4;
const uint8_t PIN_RESET_BUTTON    = 5;

const uint8_t PIN_BATT_SENSE      = A0;
const uint8_t PIN_FORCE_UNLOAD    = A6;
const uint8_t PIN_TANK_PRESSURE   = A7;

// MOSFET outputs, active HIGH
const uint8_t PIN_FAULT_MOSFET    = 6;
const uint8_t PIN_MASTER_MOSFET   = 8;
const uint8_t PIN_START_MOSFET    = 9;

// Individual relay-module outputs, active LOW
const uint8_t PIN_UNLOADER_RELAY  = A3;
const uint8_t PIN_IDLE_RELAY      = A1;
const uint8_t PIN_KILL_RELAY      = A2;

// CAN
const uint8_t PIN_CAN_CS          = 10;

// ---------------------- Output helpers ----------------------

const uint8_t RELAY_BOARD_ON  = LOW;
const uint8_t RELAY_BOARD_OFF = HIGH;

void setRelayBoard(uint8_t pin, bool on) {
  digitalWrite(pin, on ? RELAY_BOARD_ON : RELAY_BOARD_OFF);
}

void setMosfet(uint8_t pin, bool on) {
  digitalWrite(pin, on ? HIGH : LOW);
}

// ---------------------- Test / behavior switches ----------------------

// Intentionally disabled during Phase 0 troubleshooting.
// Re-enable after start/run/stop behavior is proven stable.
const bool ENABLE_PRESSURE_AUTO_STOP = false;

// Bench-test safety: a 13.8 V supply/charger can look like alternator voltage.
// While false, fresh CAN RPM is the only accepted engine-running indication.
const bool ENABLE_CHARGE_VOLTAGE_RUN_FALLBACK = false;

// ---------------------- Timing constants ----------------------

const unsigned long PRESSURE_CALL_CONFIRM_MS = 1000;
const unsigned long PRESSURE_FULL_CONFIRM_MS = 3000;

const unsigned long MASTER_ON_DELAY_MS       = 4000;
const unsigned long PRECRANK_UNLOAD_MS       = 1000;
const unsigned long START_PULSE_MS           = 750;
const unsigned long START_TIMEOUT_MS         = 30000;

const unsigned long OEM_RUN_UNLOADED_MS      = 15000;
const unsigned long STOP_UNLOAD_MS           = 8000;
const unsigned long MASTER_OFF_DELAY_AFTER_STOP_PULSE_MS = 6000;

const unsigned long RPM_STALE_MS             = 1500;
const unsigned long ENGINE_LOST_CONFIRM_MS   = 3000;

const uint16_t ENGINE_RUNNING_RPM            = 400;
const uint16_t MAX_ACCEPTED_RPM              = 4000;

const uint16_t EMERGENCY_KILL_AVG_RPM        = 3500;
const unsigned long RPM_AVG_SAMPLE_MS        = 500;
const uint8_t RPM_AVG_SAMPLE_COUNT           = 20;

const float ENGINE_RUNNING_CHARGE_VOLTAGE    = 13.2;
const unsigned long CHARGE_RUN_CONFIRM_MS    = 2000;

const unsigned long DISPLAY_UPDATE_MS        = 250;
const unsigned long SERIAL_STATUS_MS         = 1000;
const unsigned long FRAM_SAVE_MS             = 60000;

// ---------------------- Analog calibration ----------------------

const float ADC_REF_V = 5.0;
const float BATT_DIVIDER_FACTOR = (100.0 + 33.0) / 33.0;
const float BATT_CAL = 1.067;

const float PRESSURE_SENSOR_MIN_V = 0.5;
const float PRESSURE_SENSOR_SPAN_V = 4.0;
const float PRESSURE_SENSOR_MAX_PSI = 200.0;
const float PRESSURE_ALPHA = 0.15;

// ---------------------- CAN / OLED / FRAM ----------------------

MCP2515 mcp2515(PIN_CAN_CS);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
bool framPresent = false;

const uint32_t FRAM_MAGIC = 0x4E535431UL;
const uint16_t FRAM_ADDR_MAGIC   = 0;
const uint16_t FRAM_ADDR_STARTS  = 4;
const uint16_t FRAM_ADDR_RUNTIME = 8;
const uint32_t RPM_CAN_ID = 0x0C665500UL;

// ---------------------- State machine ----------------------

enum State {
  STATE_WAITING = 0,
  STATE_MASTER_ON_DELAY,
  STATE_PRECRANK_UNLOAD,
  STATE_STARTING,
  STATE_OEM_RUN,
  STATE_RUNNING_LOADED,
  STATE_STOP_UNLOAD,
  STATE_STOPPING,
  STATE_FAULT
};

enum FaultCode {
  FAULT_NONE = 0,
  FAULT_MASTER_OFF = 1,
  FAULT_START_FAIL = 2,
  FAULT_STOP_FAIL = 3,
  FAULT_ENGINE_LOST = 4,
  FAULT_OVERSPEED = 5
};

enum StopReason {
  STOP_NONE = 0,
  STOP_PRESSURE,
  STOP_ENGINE_LOST,
  STOP_FAULT,
  RELEASE_AUTO
};

State state = STATE_WAITING;
FaultCode faultCode = FAULT_NONE;
StopReason stopReason = STOP_NONE;

unsigned long stateEnteredMs = 0;
unsigned long oemRunStartMs = 0;
unsigned long stopPulseEndedMs = 0;
bool emergencyKillActive = false;

// ---------------------- Inputs / measurements ----------------------

bool autoOn = false;
bool pressureCallRaw = false;
bool pressureCallConfirmed = false;
bool pressureFullConfirmed = false;
bool masterMonitorOn = false;
bool forceUnload = false;
bool resetPressed = false;

unsigned long pressureCallStartedMs = 0;
unsigned long pressureFullStartedMs = 0;

float batteryVoltage = 0.0;
float tankPressurePsi = 0.0;
float tankPressureFiltered = 0.0;
bool pressureFilterInitialized = false;

unsigned long chargeRunStartedMs = 0;
bool chargeRunConfirmed = false;

// ---------------------- RPM / CAN ----------------------

uint16_t engineRpm = 0;
unsigned long lastRpmFrameMs = 0;
uint32_t canAnyFrameCount = 0;
uint32_t canRpmFrameCount = 0;
uint32_t canRejectedRpmCount = 0;
uint16_t lastRejectedRpm = 0;

uint16_t rpmSamples[RPM_AVG_SAMPLE_COUNT];
uint8_t rpmSampleIndex = 0;
uint8_t rpmSampleFilled = 0;
unsigned long lastRpmSampleMs = 0;
uint16_t avgRpm10s = 0;

bool wasEngineRunning = false;
unsigned long engineLostStartedMs = 0;

// ---------------------- Pulse handling ----------------------

bool startStopPulseActive = false;
unsigned long startStopPulseStartedMs = 0;

// ---------------------- Counters ----------------------

uint32_t startCount = 0;
uint32_t runtimeSeconds = 0;

unsigned long lastRuntimeTickMs = 0;
unsigned long lastFramSaveMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastSerialMs = 0;

// ================================================================
// Names
// ================================================================

const char* stateName(State s) {
  switch (s) {
    case STATE_WAITING:          return "WAIT";
    case STATE_MASTER_ON_DELAY:  return "M-ON";
    case STATE_PRECRANK_UNLOAD:  return "UNLD";
    case STATE_STARTING:         return "STRT";
    case STATE_OEM_RUN:          return "OEM";
    case STATE_RUNNING_LOADED:   return "RUN";
    case STATE_STOP_UNLOAD:      return "S-UN";
    case STATE_STOPPING:         return "STOP";
    case STATE_FAULT:            return "FLT";
    default:                     return "UNK";
  }
}

const char* faultName(FaultCode f) {
  switch (f) {
    case FAULT_NONE:        return "NONE";
    case FAULT_MASTER_OFF:  return "MASTER";
    case FAULT_START_FAIL:  return "START";
    case FAULT_STOP_FAIL:   return "STOP";
    case FAULT_ENGINE_LOST: return "LOST";
    case FAULT_OVERSPEED:   return "OVRSPD";
    default:                return "UNK";
  }
}

const char* stopReasonName(StopReason r) {
  switch (r) {
    case STOP_NONE:        return "NONE";
    case STOP_PRESSURE:    return "PRES";
    case STOP_ENGINE_LOST: return "LOST";
    case STOP_FAULT:       return "FAULT";
    case RELEASE_AUTO:     return "AUTO";
    default:               return "UNK";
  }
}

// ================================================================
// Engine-running logic
// ================================================================

bool engineRunningByRpm() {
  return (engineRpm >= ENGINE_RUNNING_RPM) &&
         ((millis() - lastRpmFrameMs) <= RPM_STALE_MS);
}

bool engineRunningByChargingVoltage() {
  return ENABLE_CHARGE_VOLTAGE_RUN_FALLBACK && chargeRunConfirmed;
}

bool engineRunning() {
  return engineRunningByRpm() || engineRunningByChargingVoltage();
}

void updateChargeRunConfirm() {
  unsigned long now = millis();

  if (batteryVoltage >= ENGINE_RUNNING_CHARGE_VOLTAGE) {
    if (chargeRunStartedMs == 0) chargeRunStartedMs = now;
    chargeRunConfirmed = (now - chargeRunStartedMs >= CHARGE_RUN_CONFIRM_MS);
  } else {
    chargeRunStartedMs = 0;
    chargeRunConfirmed = false;
  }
}

// ================================================================
// State helpers
// ================================================================

void enterState(State newState) {
  if (state != newState) {
    Serial.print(F("EVENT: ENTER "));
    Serial.println(stateName(newState));
  }
  state = newState;
  stateEnteredMs = millis();
}

void setFault(FaultCode f) {
  faultCode = f;
  stopReason = STOP_FAULT;
  if (f == FAULT_OVERSPEED) emergencyKillActive = true;

  Serial.print(F("EVENT: FAULT "));
  Serial.println(faultName(f));
  enterState(STATE_FAULT);
}

void beginStartStopPulse() {
  startStopPulseActive = true;
  startStopPulseStartedMs = millis();
  Serial.println(F("EVENT: START/STOP PULSE"));
}

void updateStartStopPulse() {
  if (startStopPulseActive &&
      (millis() - startStopPulseStartedMs >= START_PULSE_MS)) {
    startStopPulseActive = false;
    stopPulseEndedMs = millis();
    Serial.println(F("EVENT: START/STOP PULSE END"));
  }
}

// ================================================================
// FRAM
// ================================================================

uint32_t framReadU32(uint16_t addr) {
  uint32_t v = 0;
  v |= ((uint32_t)fram.read(addr + 0)) << 0;
  v |= ((uint32_t)fram.read(addr + 1)) << 8;
  v |= ((uint32_t)fram.read(addr + 2)) << 16;
  v |= ((uint32_t)fram.read(addr + 3)) << 24;
  return v;
}

void framWriteU32(uint16_t addr, uint32_t v) {
  fram.write(addr + 0, (uint8_t)((v >> 0) & 0xFF));
  fram.write(addr + 1, (uint8_t)((v >> 8) & 0xFF));
  fram.write(addr + 2, (uint8_t)((v >> 16) & 0xFF));
  fram.write(addr + 3, (uint8_t)((v >> 24) & 0xFF));
}

void loadFram() {
  framPresent = fram.begin(0x50);
  if (!framPresent) {
    Serial.println(F("FRAM: not found"));
    return;
  }

  Serial.println(F("FRAM: found"));
  uint32_t magic = framReadU32(FRAM_ADDR_MAGIC);
  if (magic != FRAM_MAGIC) {
    Serial.println(F("FRAM: init"));
    framWriteU32(FRAM_ADDR_MAGIC, FRAM_MAGIC);
    framWriteU32(FRAM_ADDR_STARTS, 0);
    framWriteU32(FRAM_ADDR_RUNTIME, 0);
  }

  startCount = framReadU32(FRAM_ADDR_STARTS);
  runtimeSeconds = framReadU32(FRAM_ADDR_RUNTIME);
}

void saveFram() {
  if (!framPresent) return;
  framWriteU32(FRAM_ADDR_STARTS, startCount);
  framWriteU32(FRAM_ADDR_RUNTIME, runtimeSeconds);
}

// ================================================================
// Inputs / analog / CAN
// ================================================================

uint16_t analogReadAveraged(uint8_t pin, uint8_t samples = 16) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(250);
  }
  return (uint16_t)(sum / samples);
}

void readInputs() {
  unsigned long now = millis();

  pressureCallRaw = (digitalRead(PIN_PRESSURE_SWITCH) == LOW);
  autoOn = (digitalRead(PIN_AUTO_SWITCH) == LOW);
  masterMonitorOn = (digitalRead(PIN_MASTER_MONITOR) == LOW);
  resetPressed = (digitalRead(PIN_RESET_BUTTON) == LOW);

  int forceRaw = analogRead(PIN_FORCE_UNLOAD);
  forceUnload = (forceRaw < 600);

  if (pressureCallRaw) {
    if (pressureCallStartedMs == 0) pressureCallStartedMs = now;
    pressureCallConfirmed = (now - pressureCallStartedMs >= PRESSURE_CALL_CONFIRM_MS);
  } else {
    pressureCallStartedMs = 0;
    pressureCallConfirmed = false;
  }

  if (!pressureCallRaw) {
    if (pressureFullStartedMs == 0) pressureFullStartedMs = now;
    pressureFullConfirmed = (now - pressureFullStartedMs >= PRESSURE_FULL_CONFIRM_MS);
  } else {
    pressureFullStartedMs = 0;
    pressureFullConfirmed = false;
  }
}

void readBatteryVoltage() {
  uint16_t raw = analogReadAveraged(PIN_BATT_SENSE);
  float adcV = ((float)raw / 1023.0) * ADC_REF_V;
  batteryVoltage = adcV * BATT_DIVIDER_FACTOR * BATT_CAL;
}

void readTankPressure() {
  uint16_t raw = analogReadAveraged(PIN_TANK_PRESSURE);
  float voltage = ((float)raw / 1023.0) * ADC_REF_V;

  float psi = (voltage - PRESSURE_SENSOR_MIN_V) *
              (PRESSURE_SENSOR_MAX_PSI / PRESSURE_SENSOR_SPAN_V);

  if (psi < 0.0) psi = 0.0;
  if (psi > PRESSURE_SENSOR_MAX_PSI) psi = PRESSURE_SENSOR_MAX_PSI;

  tankPressurePsi = psi;

  if (!pressureFilterInitialized) {
    tankPressureFiltered = psi;
    pressureFilterInitialized = true;
  } else {
    tankPressureFiltered =
      (PRESSURE_ALPHA * psi) + ((1.0 - PRESSURE_ALPHA) * tankPressureFiltered);
  }
}

void readCanRpm() {
  struct can_frame frame;

  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    canAnyFrameCount++;

    bool extended = (frame.can_id & CAN_EFF_FLAG);
    uint32_t cleanId = extended ? (frame.can_id & CAN_EFF_MASK)
                                : (frame.can_id & CAN_SFF_MASK);

    if (extended && cleanId == RPM_CAN_ID && frame.can_dlc >= 4) {
      uint16_t candidateRpm =
        ((uint16_t)frame.data[2] << 8) | frame.data[3];

      if (candidateRpm <= MAX_ACCEPTED_RPM) {
        engineRpm = candidateRpm;
        lastRpmFrameMs = millis();
        canRpmFrameCount++;
      } else {
        canRejectedRpmCount++;
        lastRejectedRpm = candidateRpm;
        Serial.print(F("EVENT: RPM REJECT "));
        Serial.println(candidateRpm);
      }
    }
  }

  if ((millis() - lastRpmFrameMs) > RPM_STALE_MS) {
    engineRpm = 0;
  }
}

void updateRpmAverage() {
  unsigned long now = millis();
  if (now - lastRpmSampleMs < RPM_AVG_SAMPLE_MS) return;
  lastRpmSampleMs = now;

  rpmSamples[rpmSampleIndex] = engineRunningByRpm() ? engineRpm : 0;
  rpmSampleIndex = (rpmSampleIndex + 1) % RPM_AVG_SAMPLE_COUNT;
  if (rpmSampleFilled < RPM_AVG_SAMPLE_COUNT) rpmSampleFilled++;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < rpmSampleFilled; i++) sum += rpmSamples[i];

  avgRpm10s = rpmSampleFilled ? (uint16_t)(sum / rpmSampleFilled) : 0;

  if (rpmSampleFilled >= RPM_AVG_SAMPLE_COUNT &&
      avgRpm10s >= EMERGENCY_KILL_AVG_RPM &&
      faultCode != FAULT_OVERSPEED) {
    setFault(FAULT_OVERSPEED);
  }
}

void updateRuntimeCounter() {
  unsigned long now = millis();

  if (lastRuntimeTickMs == 0) {
    lastRuntimeTickMs = now;
    return;
  }

  if (now - lastRuntimeTickMs >= 1000) {
    lastRuntimeTickMs += 1000;
    if (engineRunning()) runtimeSeconds++;
  }

  if (now - lastFramSaveMs >= FRAM_SAVE_MS) {
    lastFramSaveMs = now;
    saveFram();
  }
}

// ================================================================
// Fault blink output
// ================================================================

bool faultBlinkOutputOn() {
  if (faultCode == FAULT_NONE) return false;

  const unsigned long onMs = 250;
  const unsigned long offMs = 250;
  const unsigned long pauseMs = 1500;

  uint8_t count = (uint8_t)faultCode;
  unsigned long patternMs = (count * (onMs + offMs)) + pauseMs;
  unsigned long t = millis() % patternMs;

  for (uint8_t i = 0; i < count; i++) {
    unsigned long start = i * (onMs + offMs);
    if (t >= start && t < start + onMs) return true;
  }
  return false;
}

// ================================================================
// State machine
// ================================================================

void releaseControlForAutoOff() {
  if (state != STATE_WAITING) Serial.println(F("EVENT: AUTO OFF RELEASE"));
  stopReason = RELEASE_AUTO;
  startStopPulseActive = false;
  if (state != STATE_FAULT) enterState(STATE_WAITING);
}

void updateStateMachine() {
  unsigned long now = millis();
  bool running = engineRunning();

  if (resetPressed) {
    if (faultCode != FAULT_NONE) Serial.println(F("EVENT: FAULT CLEAR"));
    faultCode = FAULT_NONE;
    emergencyKillActive = false;
    stopReason = STOP_NONE;
    if (state == STATE_FAULT) enterState(STATE_WAITING);
  }

  if (!autoOn) {
    releaseControlForAutoOff();
    return;
  }

  bool shouldBeRunning =
    (state == STATE_OEM_RUN ||
     state == STATE_RUNNING_LOADED ||
     state == STATE_STOP_UNLOAD ||
     state == STATE_STOPPING);

  if (shouldBeRunning) {
    if (!running && wasEngineRunning) {
      if (engineLostStartedMs == 0) engineLostStartedMs = now;
      if (now - engineLostStartedMs >= ENGINE_LOST_CONFIRM_MS) {
        stopReason = STOP_ENGINE_LOST;
        setFault(FAULT_ENGINE_LOST);
        return;
      }
    } else {
      engineLostStartedMs = 0;
    }
  } else {
    engineLostStartedMs = 0;
  }

  wasEngineRunning = running;
  if (state == STATE_FAULT) return;

  switch (state) {
    case STATE_WAITING:
      stopReason = STOP_NONE;
      if (pressureCallConfirmed && !forceUnload) {
        Serial.println(F("EVENT: PRESSURE CALL CONFIRMED"));
        enterState(STATE_MASTER_ON_DELAY);
      }
      break;

    case STATE_MASTER_ON_DELAY:
      if (now - stateEnteredMs >= MASTER_ON_DELAY_MS) {
        if (!masterMonitorOn) {
          setFault(FAULT_MASTER_OFF);
          return;
        }
        Serial.println(F("EVENT: MASTER CONFIRMED"));
        enterState(STATE_PRECRANK_UNLOAD);
      }
      break;

    case STATE_PRECRANK_UNLOAD:
      if (now - stateEnteredMs >= PRECRANK_UNLOAD_MS) {
        beginStartStopPulse();
        startCount++;
        saveFram();
        enterState(STATE_STARTING);
      }
      break;

    case STATE_STARTING:
      if (running) {
        Serial.println(F("EVENT: ENGINE RUNNING"));
        oemRunStartMs = now;
        enterState(STATE_OEM_RUN);
      } else if (now - stateEnteredMs >= START_TIMEOUT_MS) {
        setFault(FAULT_START_FAIL);
      }
      break;

    case STATE_OEM_RUN:
      if (now - oemRunStartMs >= OEM_RUN_UNLOADED_MS) {
        Serial.println(F("EVENT: LOAD COMPRESSOR"));
        enterState(STATE_RUNNING_LOADED);
      }
      break;

    case STATE_RUNNING_LOADED:
      if (ENABLE_PRESSURE_AUTO_STOP && pressureFullConfirmed) {
        Serial.println(F("EVENT: PRESSURE FULL CONFIRMED"));
        stopReason = STOP_PRESSURE;
        enterState(STATE_STOP_UNLOAD);
      }
      break;

    case STATE_STOP_UNLOAD:
      if (!running) {
        enterState(STATE_WAITING);
      } else if (now - stateEnteredMs >= STOP_UNLOAD_MS) {
        beginStartStopPulse();
        enterState(STATE_STOPPING);
      }
      break;

    case STATE_STOPPING:
      if (!startStopPulseActive &&
          (now - stopPulseEndedMs >= MASTER_OFF_DELAY_AFTER_STOP_PULSE_MS)) {
        Serial.println(F("EVENT: MASTER OFF AFTER STOP"));
        enterState(STATE_WAITING);
      }
      break;

    case STATE_FAULT:
    default:
      break;
  }
}

// ================================================================
// Outputs
// ================================================================

void applyOutputs() {
  bool running = engineRunning();

  bool masterOn = false;
  bool startStopOn = startStopPulseActive;
  bool unloaderOn = false;
  bool idleOn = false;
  bool killOn = false;
  bool faultOutOn = faultBlinkOutputOn();

  if (!autoOn) {
    masterOn = false;
    startStopOn = false;
    unloaderOn = forceUnload;
    idleOn = false;
    killOn = false;

    setMosfet(PIN_MASTER_MOSFET, masterOn);
    setMosfet(PIN_START_MOSFET, startStopOn);
    setMosfet(PIN_FAULT_MOSFET, faultOutOn);
    setRelayBoard(PIN_UNLOADER_RELAY, unloaderOn);
    setRelayBoard(PIN_IDLE_RELAY, idleOn);
    setRelayBoard(PIN_KILL_RELAY, killOn);
    return;
  }

  switch (state) {
    case STATE_WAITING:
      masterOn = false;
      unloaderOn = forceUnload;
      break;

    case STATE_MASTER_ON_DELAY:
      masterOn = true;
      unloaderOn = forceUnload;
      break;

    case STATE_PRECRANK_UNLOAD:
    case STATE_STARTING:
    case STATE_OEM_RUN:
      masterOn = true;
      unloaderOn = true;
      break;

    case STATE_RUNNING_LOADED:
      masterOn = true;
      unloaderOn = forceUnload;
      break;

    case STATE_STOP_UNLOAD:
    case STATE_STOPPING:
      masterOn = true;
      unloaderOn = true;
      break;

    case STATE_FAULT:
      masterOn = running && !emergencyKillActive;
      unloaderOn = running || forceUnload;
      break;
  }

  if (emergencyKillActive) {
    killOn = true;
    unloaderOn = true;
  }

  // Idle is intentionally unused in the normal Phase 0 sequence.
  idleOn = false;

  setMosfet(PIN_MASTER_MOSFET, masterOn);
  setMosfet(PIN_START_MOSFET, startStopOn);
  setMosfet(PIN_FAULT_MOSFET, faultOutOn);

  setRelayBoard(PIN_UNLOADER_RELAY, unloaderOn);
  setRelayBoard(PIN_IDLE_RELAY, idleOn);
  setRelayBoard(PIN_KILL_RELAY, killOn);
}

// ================================================================
// Display / serial
// ================================================================

void updateDisplay() {
  if (millis() - lastDisplayMs < DISPLAY_UPDATE_MS) return;
  lastDisplayMs = millis();

  char line[24];
  char battStr[8];
  char hobbsStr[12];
  char cycleStr[12];

  // Avoid AVR floating-point snprintf.
  uint32_t hobbsTenths = runtimeSeconds / 360UL;
  uint32_t hobbsHours = hobbsTenths / 10UL;
  uint8_t hobbsDecimal = hobbsTenths % 10UL;

  snprintf(hobbsStr, sizeof(hobbsStr),
           "H:%lu.%u",
           (unsigned long)hobbsHours,
           hobbsDecimal);

  snprintf(cycleStr, sizeof(cycleStr),
           "C:%lu",
           (unsigned long)startCount);

  u8g2.clearBuffer();

  // Top left: machine state / fault.
  u8g2.setFont(u8g2_font_5x8_tf);

  if (faultCode != FAULT_NONE) {
    snprintf(line, sizeof(line), "FLT:%s", faultName(faultCode));
  } else {
    snprintf(line, sizeof(line), "%s AUTO:%s",
             stateName(state),
             autoOn ? "ON" : "OFF");
  }

  u8g2.drawStr(0, 8, line);

  // Top right: Hobbs.
  int hobbsWidth = u8g2.getStrWidth(hobbsStr);
  u8g2.drawStr(128 - hobbsWidth, 8, hobbsStr);

  // Center: large live CAN RPM.
  u8g2.setFont(u8g2_font_helvB14_tf);

  if (engineRpm > 0 && engineRunningByRpm()) {
    snprintf(line, sizeof(line), "%4u RPM", engineRpm);
  } else {
    snprintf(line, sizeof(line), "---- RPM");
  }

  u8g2.drawStr(0, 24, line);

  // Bottom left: pressure + battery.
  u8g2.setFont(u8g2_font_5x8_tf);

  dtostrf(batteryVoltage, 0, 1, battStr);

  snprintf(line, sizeof(line),
           "%3u PSI %sV",
           (unsigned int)(tankPressureFiltered + 0.5),
           battStr);

  u8g2.drawStr(0, 32, line);

  // Bottom right: cycle count.
  int cycleWidth = u8g2.getStrWidth(cycleStr);
  u8g2.drawStr(128 - cycleWidth, 32, cycleStr);

  u8g2.sendBuffer();
}

void printSerialStatus() {
  if (millis() - lastSerialMs < SERIAL_STATUS_MS) return;
  lastSerialMs = millis();

  Serial.print(F("STATUS "));
  Serial.print(F("STATE=")); Serial.print(stateName(state));
  Serial.print(F(" AUTO=")); Serial.print(autoOn ? F("ON") : F("OFF"));
  Serial.print(F(" PRAW=")); Serial.print(pressureCallRaw ? F("CALL") : F("FULL"));
  Serial.print(F(" PCALL=")); Serial.print(pressureCallConfirmed ? F("Y") : F("N"));
  Serial.print(F(" PFULL=")); Serial.print(pressureFullConfirmed ? F("Y") : F("N"));
  Serial.print(F(" MASTERMON=")); Serial.print(masterMonitorOn ? F("ON") : F("OFF"));
  Serial.print(F(" FRC_UNLD=")); Serial.print(forceUnload ? F("Y") : F("N"));
  Serial.print(F(" RPM=")); Serial.print(engineRpm);
  Serial.print(F(" AVG10=")); Serial.print(avgRpm10s);
  Serial.print(F(" RUN_RPM=")); Serial.print(engineRunningByRpm() ? F("Y") : F("N"));
  Serial.print(F(" RUN_V=")); Serial.print(engineRunningByChargingVoltage() ? F("Y") : F("N"));
  Serial.print(F(" PSI=")); Serial.print(tankPressureFiltered, 1);
  Serial.print(F(" BATT=")); Serial.print(batteryVoltage, 2);
  Serial.print(F(" FAULT=")); Serial.print(faultName(faultCode));
  Serial.print(F(" STOP=")); Serial.print(stopReasonName(stopReason));
  Serial.print(F(" STARTS=")); Serial.print(startCount);
  Serial.print(F(" HRS=")); Serial.print(runtimeSeconds / 3600.0, 2);
  Serial.print(F(" CANANY=")); Serial.print(canAnyFrameCount);
  Serial.print(F(" CANRPM=")); Serial.print(canRpmFrameCount);
  Serial.print(F(" CANREJ=")); Serial.print(canRejectedRpmCount);
  Serial.print(F(" LASTBAD=")); Serial.println(lastRejectedRpm);
}

// ================================================================
// Setup / loop
// ================================================================

void setup() {
  // Force all outputs safe before initializing anything else.
  digitalWrite(PIN_FAULT_MOSFET, LOW);
  digitalWrite(PIN_MASTER_MOSFET, LOW);
  digitalWrite(PIN_START_MOSFET, LOW);

  digitalWrite(PIN_UNLOADER_RELAY, RELAY_BOARD_OFF);
  digitalWrite(PIN_IDLE_RELAY, RELAY_BOARD_OFF);
  digitalWrite(PIN_KILL_RELAY, RELAY_BOARD_OFF);

  pinMode(PIN_FAULT_MOSFET, OUTPUT);
  pinMode(PIN_MASTER_MOSFET, OUTPUT);
  pinMode(PIN_START_MOSFET, OUTPUT);

  pinMode(PIN_UNLOADER_RELAY, OUTPUT);
  pinMode(PIN_IDLE_RELAY, OUTPUT);
  pinMode(PIN_KILL_RELAY, OUTPUT);

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("NorthStar Compressor Controller - Phase 0 Hardware Rebuild"));

  pinMode(PIN_PRESSURE_SWITCH, INPUT_PULLUP);
  pinMode(PIN_MASTER_MONITOR, INPUT_PULLUP);
  pinMode(PIN_AUTO_SWITCH, INPUT_PULLUP);
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);

  pinMode(PIN_BATT_SENSE, INPUT);
  pinMode(PIN_FORCE_UNLOAD, INPUT);
  pinMode(PIN_TANK_PRESSURE, INPUT);

  Wire.begin();

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 8, "NorthStar Ctrl");
  u8g2.drawStr(0, 16, "Phase 0 Rebuild");
  u8g2.drawStr(0, 24, "D6/D8/D9 MOSFET");
  u8g2.sendBuffer();

  loadFram();

  SPI.begin();
  mcp2515.reset();

  MCP2515::ERROR canSpeedResult =
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);

  if (canSpeedResult == MCP2515::ERROR_OK) {
    Serial.println(F("CAN: bitrate OK"));
  } else {
    Serial.println(F("CAN: bitrate FAIL"));
  }

  mcp2515.setNormalMode();
  Serial.println(F("CAN: normal mode"));

  for (uint8_t i = 0; i < RPM_AVG_SAMPLE_COUNT; i++) rpmSamples[i] = 0;

  stateEnteredMs = millis();
  lastRuntimeTickMs = millis();
  lastFramSaveMs = millis();

  Serial.println(F("READY"));
}

void loop() {
  updateStartStopPulse();

  readCanRpm();
  readInputs();
  readBatteryVoltage();
  readTankPressure();
  updateChargeRunConfirm();

  updateRpmAverage();
  updateRuntimeCounter();

  updateStateMachine();
  applyOutputs();

  updateDisplay();
  printSerialStatus();
}
