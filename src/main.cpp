#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>
#include <U8g2lib.h>
#include <Adafruit_FRAM_I2C.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// NorthStar compressor controller - compact Phase 0 field build

// Pins
const uint8_t PIN_PRESSURE_SWITCH = 2;
const uint8_t PIN_MASTER_MONITOR  = 3;
const uint8_t PIN_AUTO_SWITCH     = 4;
const uint8_t PIN_RESET_BUTTON    = 5;
const uint8_t PIN_FAULT_MOSFET    = 6;
const uint8_t PIN_MASTER_MOSFET   = 8;
const uint8_t PIN_START_MOSFET    = 9;
const uint8_t PIN_CAN_CS          = 10;
const uint8_t PIN_BATT_SENSE      = A0;
const uint8_t PIN_IDLE_RELAY      = A1;
const uint8_t PIN_KILL_RELAY      = A2;
const uint8_t PIN_UNLOADER_RELAY  = A3;
const uint8_t PIN_FORCE_UNLOAD    = A6;
const uint8_t PIN_TANK_PRESSURE   = A7;

const uint8_t RELAY_ON = LOW;
const uint8_t RELAY_OFF = HIGH;
void setRelay(uint8_t pin, bool on) { digitalWrite(pin, on ? RELAY_ON : RELAY_OFF); }
void setMosfet(uint8_t pin, bool on) { digitalWrite(pin, on ? HIGH : LOW); }

// Behavior
const bool ENABLE_PRESSURE_AUTO_STOP = true;
const bool ENABLE_CHARGE_VOLTAGE_RUN_FALLBACK = false;

// Timings
const unsigned long PRESSURE_CALL_CONFIRM_MS = 1000;
const unsigned long PRESSURE_FULL_CONFIRM_MS = 3000;
const unsigned long MASTER_ON_DELAY_MS = 4000;
const unsigned long PRECRANK_UNLOAD_MS = 1000;
unsigned long startPulseMs = 1250;
// OEM engine controller owns its internal retry sequence. The engine manual
// specifies up to three automatic attempts but does not publish their timing,
// so allow a generous one-minute envelope after the initial button press.
const unsigned long STARTER_ACTIVITY_TIMEOUT_MS = 5000;
const unsigned long OEM_START_SEQUENCE_TIMEOUT_MS = 60000;
const unsigned long OEM_RUN_UNLOADED_MS = 15000;
const unsigned long STOP_UNLOAD_MS = 10000;
const unsigned long MASTER_OFF_DELAY_MS = 6000;
const unsigned long RPM_STALE_MS = 1500;
const unsigned long RPM_RUN_CONFIRM_MS = 750;
const unsigned long START_CONFIRMED_MS = 1000;
const unsigned long ENGINE_LOST_CONFIRM_MS = 3000;
const uint16_t STARTER_ACTIVITY_RPM = 100;
const uint16_t START_CONFIRMED_RPM = 800;
const uint16_t ENGINE_RUNNING_RPM = 400;
const uint16_t MAX_ACCEPTED_RPM = 4000;
const uint16_t EMERGENCY_KILL_AVG_RPM = 3500;
const unsigned long RPM_AVG_SAMPLE_MS = 500;
const uint8_t RPM_AVG_SAMPLE_COUNT = 20;
const float ENGINE_RUNNING_CHARGE_VOLTAGE = 13.2;
const unsigned long CHARGE_RUN_CONFIRM_MS = 2000;
const unsigned long DISPLAY_UPDATE_MS = 250;
const unsigned long SERIAL_STATUS_MS = 1000;
const unsigned long FRAM_SAVE_MS = 60000;

// Analog calibration
const float ADC_REF_V = 5.0;
const float BATT_DIVIDER_FACTOR = (100.0 + 33.0) / 33.0;
const float BATT_CAL = 1.067;
const float PRESSURE_SENSOR_MIN_V = 0.5;
const float PRESSURE_SENSOR_SPAN_V = 4.0;
const float PRESSURE_SENSOR_MAX_PSI = 200.0;
const float PRESSURE_ALPHA = 0.15;

// Devices. 1-page OLED mode cuts the framebuffer RAM substantially.
MCP2515 mcp2515(PIN_CAN_CS);
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
bool framPresent = false;
const uint32_t FRAM_MAGIC = 0x4E535431UL;
const uint16_t FRAM_ADDR_MAGIC = 0;
const uint16_t FRAM_ADDR_STARTS = 4;
const uint16_t FRAM_ADDR_RUNTIME = 8;
const uint16_t FRAM_ADDR_REMOTE_MAGIC = 12;
const uint16_t FRAM_ADDR_REMOTE_AUTO = 16;
const uint32_t FRAM_REMOTE_MAGIC = 0x524D5431UL;  // "RMT1"
const uint32_t RPM_CAN_ID = 0x0C665500UL;

enum State {
  STATE_WAITING = 0, STATE_MASTER_ON_DELAY, STATE_PRECRANK_UNLOAD,
  STATE_STARTING, STATE_OEM_RUN, STATE_RUNNING_LOADED,
  STATE_STOP_UNLOAD, STATE_STOPPING, STATE_FAULT
};
enum FaultCode {
  FAULT_NONE = 0, FAULT_MASTER_OFF = 1, FAULT_START_FAIL = 2,
  FAULT_STOP_FAIL = 3, FAULT_ENGINE_LOST = 4, FAULT_OVERSPEED = 5
};
enum StopReason { STOP_NONE = 0, STOP_PRESSURE, STOP_ENGINE_LOST, STOP_FAULT, RELEASE_AUTO };

State state = STATE_WAITING;
FaultCode faultCode = FAULT_NONE;
StopReason stopReason = STOP_NONE;
unsigned long stateEnteredMs = 0;
unsigned long oemRunStartMs = 0;
unsigned long stopPulseEndedMs = 0;
bool emergencyKillActive = false;

// Inputs / measurements
bool autoOn = false;                  // physical AUTO/OFF switch
bool remoteAutoPermit = true;         // ESP32 may inhibit AUTO, never force a start
bool physicalAutoInitialized = false;
bool lastPhysicalAutoOn = false;
bool pressureCallRaw = false;
bool pressureCallConfirmed = false;
bool pressureFullConfirmed = false;
bool masterMonitorOn = false;
bool forceUnload = false;
bool resetPressed = false;
unsigned long pressureCallStartedMs = 0;
unsigned long pressureFullStartedMs = 0;
float batteryVoltage = 0.0;
float tankPressureFiltered = 0.0;
bool pressureFilterInitialized = false;
unsigned long chargeRunStartedMs = 0;
bool chargeRunConfirmed = false;

// RPM / CAN
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
unsigned long rpmRunStartedMs = 0;
bool rpmRunConfirmed = false;
unsigned long engineLostStartedMs = 0;

// OEM start-sequence supervision
bool starterActivitySeen = false;
unsigned long startConfirmedStartedMs = 0;

// Automatic pulse
bool startStopPulseActive = false;
unsigned long startStopPulseStartedMs = 0;

// Field manual overrides
bool fieldManualMode = false;
bool manualMasterOn = false;
bool manualStartStopOn = false;
bool manualUnloaderOn = false;
bool manualIdleOn = false;
bool manualKillOn = false;
bool actualMasterOn = false;
bool actualStartStopOn = false;
bool actualUnloaderOn = false;
bool actualIdleOn = false;
bool actualKillOn = false;
char serialCommandBuffer[40];
uint8_t serialCommandLength = 0;

// Counters
uint32_t startCount = 0;
uint32_t runtimeSeconds = 0;
unsigned long lastRuntimeTickMs = 0;
unsigned long lastFramSaveMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastSerialMs = 0;

const char* stateName(State s) {
  switch (s) {
    case STATE_WAITING: return "WAIT";
    case STATE_MASTER_ON_DELAY: return "M-ON";
    case STATE_PRECRANK_UNLOAD: return "UNLD";
    case STATE_STARTING: return "STRT";
    case STATE_OEM_RUN: return "OEM";
    case STATE_RUNNING_LOADED: return "RUN";
    case STATE_STOP_UNLOAD: return "S-UN";
    case STATE_STOPPING: return "STOP";
    case STATE_FAULT: return "FLT";
    default: return "UNK";
  }
}

const char* faultName(FaultCode f) {
  switch (f) {
    case FAULT_NONE: return "NONE";
    case FAULT_MASTER_OFF: return "MASTER";
    case FAULT_START_FAIL: return "START";
    case FAULT_STOP_FAIL: return "STOP";
    case FAULT_ENGINE_LOST: return "LOST";
    case FAULT_OVERSPEED: return "OVRSPD";
    default: return "UNK";
  }
}

bool rpmSignalPresent() {
  return engineRpm >= ENGINE_RUNNING_RPM && (millis() - lastRpmFrameMs) <= RPM_STALE_MS;
}
bool engineRunningByRpm() { return rpmRunConfirmed; }
bool engineRunningByChargingVoltage() {
  return ENABLE_CHARGE_VOLTAGE_RUN_FALLBACK && chargeRunConfirmed;
}
bool engineRunning() { return engineRunningByRpm() || engineRunningByChargingVoltage(); }

void updateChargeRunConfirm() {
  unsigned long now = millis();
  if (batteryVoltage >= ENGINE_RUNNING_CHARGE_VOLTAGE) {
    if (!chargeRunStartedMs) chargeRunStartedMs = now;
    chargeRunConfirmed = (now - chargeRunStartedMs >= CHARGE_RUN_CONFIRM_MS);
  } else {
    chargeRunStartedMs = 0;
    chargeRunConfirmed = false;
  }
}

void enterState(State s) {
  if (state != s) {
    Serial.print(F("EV ENTER "));
    Serial.println(stateName(s));
  }
  state = s;
  stateEnteredMs = millis();
}

void setFault(FaultCode f) {
  faultCode = f;
  stopReason = STOP_FAULT;
  if (f == FAULT_OVERSPEED) emergencyKillActive = true;
  Serial.print(F("EV FAULT "));
  Serial.println(faultName(f));
  enterState(STATE_FAULT);
}

void beginStartStopPulse() {
  startStopPulseActive = true;
  startStopPulseStartedMs = millis();
  Serial.print(F("EV PULSE "));
  Serial.println(startPulseMs);
}
void updateStartStopPulse() {
  if (startStopPulseActive && millis() - startStopPulseStartedMs >= startPulseMs) {
    startStopPulseActive = false;
    stopPulseEndedMs = millis();
    Serial.println(F("EV PULSE END"));
  }
}

uint32_t framReadU32(uint16_t a) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < 4; i++) v |= ((uint32_t)fram.read(a + i)) << (8 * i);
  return v;
}
void framWriteU32(uint16_t a, uint32_t v) {
  for (uint8_t i = 0; i < 4; i++) fram.write(a + i, (uint8_t)(v >> (8 * i)));
}
void loadFram() {
  framPresent = fram.begin(0x50);
  if (!framPresent) return;
  if (framReadU32(FRAM_ADDR_MAGIC) != FRAM_MAGIC) {
    framWriteU32(FRAM_ADDR_MAGIC, FRAM_MAGIC);
    framWriteU32(FRAM_ADDR_STARTS, 0);
    framWriteU32(FRAM_ADDR_RUNTIME, 0);
  }
  startCount = framReadU32(FRAM_ADDR_STARTS);
  runtimeSeconds = framReadU32(FRAM_ADDR_RUNTIME);

  // Separate migration marker so existing installations upgrade with remote
  // AUTO permitted instead of interpreting previously-unused FRAM as state.
  if (framReadU32(FRAM_ADDR_REMOTE_MAGIC) != FRAM_REMOTE_MAGIC) {
    framWriteU32(FRAM_ADDR_REMOTE_MAGIC, FRAM_REMOTE_MAGIC);
    fram.write(FRAM_ADDR_REMOTE_AUTO, 1);
  }
  remoteAutoPermit = fram.read(FRAM_ADDR_REMOTE_AUTO) != 0;
}
void saveFram() {
  if (!framPresent) return;
  framWriteU32(FRAM_ADDR_STARTS, startCount);
  framWriteU32(FRAM_ADDR_RUNTIME, runtimeSeconds);
}

bool effectiveAutoOn() {
  return autoOn && remoteAutoPermit;
}

void setRemoteAutoPermit(bool permit, bool localRecovery = false) {
  if (remoteAutoPermit == permit) return;

  remoteAutoPermit = permit;
  if (framPresent) fram.write(FRAM_ADDR_REMOTE_AUTO, permit ? 1 : 0);

  Serial.print(F("EV REMOTE AUTO "));
  Serial.print(permit ? F("ON") : F("OFF"));
  if (localRecovery) Serial.print(F(" LOCAL"));
  Serial.println();
}

uint16_t analogReadAveraged(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 16; i++) { sum += analogRead(pin); delayMicroseconds(250); }
  return (uint16_t)(sum >> 4);
}

void readInputs() {
  unsigned long now = millis();
  pressureCallRaw = digitalRead(PIN_PRESSURE_SWITCH) == LOW;

  const bool newPhysicalAutoOn = digitalRead(PIN_AUTO_SWITCH) == LOW;
  if (!physicalAutoInitialized) {
    physicalAutoInitialized = true;
    lastPhysicalAutoOn = newPhysicalAutoOn;
  } else {
    // A deliberate local OFF -> AUTO cycle clears a persisted remote inhibit.
    // This guarantees the machine remains locally recoverable if the ESP32,
    // MQTT, or Home Assistant is unavailable.
    if (newPhysicalAutoOn && !lastPhysicalAutoOn && !remoteAutoPermit) {
      setRemoteAutoPermit(true, true);
    }
    lastPhysicalAutoOn = newPhysicalAutoOn;
  }
  autoOn = newPhysicalAutoOn;

  masterMonitorOn = digitalRead(PIN_MASTER_MONITOR) == LOW;
  resetPressed = digitalRead(PIN_RESET_BUTTON) == LOW;
  forceUnload = analogRead(PIN_FORCE_UNLOAD) < 600;

  if (pressureCallRaw) {
    if (!pressureCallStartedMs) pressureCallStartedMs = now;
    pressureCallConfirmed = now - pressureCallStartedMs >= PRESSURE_CALL_CONFIRM_MS;
  } else {
    pressureCallStartedMs = 0;
    pressureCallConfirmed = false;
  }
  if (!pressureCallRaw) {
    if (!pressureFullStartedMs) pressureFullStartedMs = now;
    pressureFullConfirmed = now - pressureFullStartedMs >= PRESSURE_FULL_CONFIRM_MS;
  } else {
    pressureFullStartedMs = 0;
    pressureFullConfirmed = false;
  }
}

void readBatteryVoltage() {
  uint16_t raw = analogReadAveraged(PIN_BATT_SENSE);
  batteryVoltage = ((float)raw / 1023.0) * ADC_REF_V * BATT_DIVIDER_FACTOR * BATT_CAL;
}
void readTankPressure() {
  uint16_t raw = analogReadAveraged(PIN_TANK_PRESSURE);
  float v = ((float)raw / 1023.0) * ADC_REF_V;
  float psi = (v - PRESSURE_SENSOR_MIN_V) * (PRESSURE_SENSOR_MAX_PSI / PRESSURE_SENSOR_SPAN_V);
  if (psi < 0) psi = 0;
  if (psi > PRESSURE_SENSOR_MAX_PSI) psi = PRESSURE_SENSOR_MAX_PSI;
  if (!pressureFilterInitialized) { tankPressureFiltered = psi; pressureFilterInitialized = true; }
  else tankPressureFiltered = PRESSURE_ALPHA * psi + (1.0 - PRESSURE_ALPHA) * tankPressureFiltered;
}

void readCanRpm() {
  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    canAnyFrameCount++;
    bool ext = frame.can_id & CAN_EFF_FLAG;
    uint32_t id = ext ? frame.can_id & CAN_EFF_MASK : frame.can_id & CAN_SFF_MASK;
    if (ext && id == RPM_CAN_ID && frame.can_dlc >= 4) {
      uint16_t r = ((uint16_t)frame.data[2] << 8) | frame.data[3];
      if (r <= MAX_ACCEPTED_RPM) {
        engineRpm = r;
        lastRpmFrameMs = millis();
        canRpmFrameCount++;
      } else {
        canRejectedRpmCount++;
        lastRejectedRpm = r;
      }
    }
  }

  unsigned long now = millis();
  if (now - lastRpmFrameMs > RPM_STALE_MS) engineRpm = 0;

  if (rpmSignalPresent()) {
    if (!rpmRunStartedMs) rpmRunStartedMs = now;
    rpmRunConfirmed = (now - rpmRunStartedMs >= RPM_RUN_CONFIRM_MS);
  } else {
    rpmRunStartedMs = 0;
    rpmRunConfirmed = false;
  }
}

void updateRpmAverage() {
  unsigned long now = millis();
  if (now - lastRpmSampleMs < RPM_AVG_SAMPLE_MS) return;
  lastRpmSampleMs = now;
  rpmSamples[rpmSampleIndex] = rpmSignalPresent() ? engineRpm : 0;
  rpmSampleIndex = (rpmSampleIndex + 1) % RPM_AVG_SAMPLE_COUNT;
  if (rpmSampleFilled < RPM_AVG_SAMPLE_COUNT) rpmSampleFilled++;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < rpmSampleFilled; i++) sum += rpmSamples[i];
  avgRpm10s = rpmSampleFilled ? (uint16_t)(sum / rpmSampleFilled) : 0;
  if (rpmSampleFilled >= RPM_AVG_SAMPLE_COUNT && avgRpm10s >= EMERGENCY_KILL_AVG_RPM && faultCode != FAULT_OVERSPEED)
    setFault(FAULT_OVERSPEED);
}

void updateRuntimeCounter() {
  unsigned long now = millis();
  if (!lastRuntimeTickMs) { lastRuntimeTickMs = now; return; }
  if (now - lastRuntimeTickMs >= 1000) {
    lastRuntimeTickMs += 1000;
    if (engineRunning()) runtimeSeconds++;
  }
  if (now - lastFramSaveMs >= FRAM_SAVE_MS) {
    lastFramSaveMs = now;
    saveFram();
  }
}

bool faultBlinkOutputOn() {
  if (faultCode == FAULT_NONE) return false;
  uint8_t n = (uint8_t)faultCode;
  unsigned long t = millis() % (n * 500UL + 1500UL);
  for (uint8_t i = 0; i < n; i++) if (t >= i * 500UL && t < i * 500UL + 250UL) return true;
  return false;
}

void releaseControlForAutoOff() {
  stopReason = RELEASE_AUTO;
  startStopPulseActive = false;
  starterActivitySeen = false;
  startConfirmedStartedMs = 0;
  if (state != STATE_FAULT) enterState(STATE_WAITING);
}

void updateStateMachine() {
  unsigned long now = millis();
  bool running = engineRunning();

  if (resetPressed) {
    faultCode = FAULT_NONE;
    emergencyKillActive = false;
    stopReason = STOP_NONE;
    if (state == STATE_FAULT) enterState(STATE_WAITING);
  }
  if (!effectiveAutoOn()) { releaseControlForAutoOff(); return; }

  // Engine-loss supervision applies only to normal running states. During
  // STARTING the OEM engine controller owns its internal retry sequence, and
  // during STOPPING/idle-down RPM loss is either expected or harmless.
  bool shouldRun = state == STATE_OEM_RUN || state == STATE_RUNNING_LOADED;
  if (shouldRun) {
    if (!running) {
      if (!engineLostStartedMs) engineLostStartedMs = now;
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
  if (state == STATE_FAULT) return;

  switch (state) {
    case STATE_WAITING:
      stopReason = STOP_NONE;
      starterActivitySeen = false;
      startConfirmedStartedMs = 0;
      if (running) { oemRunStartMs = now; enterState(STATE_OEM_RUN); break; }
      if (pressureCallConfirmed && !forceUnload) enterState(STATE_MASTER_ON_DELAY);
      break;

    case STATE_MASTER_ON_DELAY:
      if (running) { oemRunStartMs = now; enterState(STATE_OEM_RUN); break; }
      if (now - stateEnteredMs >= MASTER_ON_DELAY_MS) enterState(STATE_PRECRANK_UNLOAD);
      break;

    case STATE_PRECRANK_UNLOAD:
      if (running) { oemRunStartMs = now; enterState(STATE_OEM_RUN); break; }
      if (now - stateEnteredMs >= PRECRANK_UNLOAD_MS) {
        starterActivitySeen = false;
        startConfirmedStartedMs = 0;
        beginStartStopPulse();
        startCount++;
        saveFram();
        enterState(STATE_STARTING);
      }
      break;

    case STATE_STARTING: {
      bool freshRpm = (now - lastRpmFrameMs) <= RPM_STALE_MS;

      // Seeing starter-speed RPM proves the OEM controller accepted the button
      // press. From that point forward, leave it alone and let it perform its
      // own up-to-three-attempt sequence.
      if (!starterActivitySeen && freshRpm && engineRpm >= STARTER_ACTIVITY_RPM) {
        starterActivitySeen = true;
        Serial.println(F("EV CRANK"));
      }

      // Do not call a brief 400-500 RPM catch a successful start. Field data
      // showed a failed attempt can live there for several seconds before the
      // OEM controller retries. Require a real climb above 800 RPM for 1 s.
      if (freshRpm && engineRpm >= START_CONFIRMED_RPM) {
        if (!startConfirmedStartedMs) startConfirmedStartedMs = now;
        if (now - startConfirmedStartedMs >= START_CONFIRMED_MS) {
          Serial.println(F("EV START CONFIRMED"));
          oemRunStartMs = now;
          enterState(STATE_OEM_RUN);
        }
      } else {
        startConfirmedStartedMs = 0;
      }

      if (state != STATE_STARTING) break;

      // If the button press never produces even starter-speed RPM, fail fast.
      // Once cranking has been observed, allow the OEM controller a full minute
      // to complete its documented three-attempt sequence.
      if (!starterActivitySeen && now - stateEnteredMs >= STARTER_ACTIVITY_TIMEOUT_MS) {
        setFault(FAULT_START_FAIL);
      } else if (starterActivitySeen && now - stateEnteredMs >= OEM_START_SEQUENCE_TIMEOUT_MS) {
        setFault(FAULT_START_FAIL);
      }
      break;
    }

    case STATE_OEM_RUN:
      if (now - oemRunStartMs >= OEM_RUN_UNLOADED_MS) enterState(STATE_RUNNING_LOADED);
      break;

    case STATE_RUNNING_LOADED:
      if (ENABLE_PRESSURE_AUTO_STOP && pressureFullConfirmed) {
        stopReason = STOP_PRESSURE;
        enterState(STATE_STOP_UNLOAD);
      }
      break;

    case STATE_STOP_UNLOAD:
      if (!running) enterState(STATE_WAITING);
      else if (now - stateEnteredMs >= STOP_UNLOAD_MS) {
        beginStartStopPulse();
        enterState(STATE_STOPPING);
      }
      break;

    case STATE_STOPPING:
      if (!startStopPulseActive && now - stopPulseEndedMs >= MASTER_OFF_DELAY_MS) enterState(STATE_WAITING);
      break;

    case STATE_FAULT: default: break;
  }
}

void writeOutputs(bool master, bool startStop, bool unload, bool idle, bool kill) {
  actualMasterOn = master;
  actualStartStopOn = startStop;
  actualUnloaderOn = unload;
  actualIdleOn = idle;
  actualKillOn = kill;
  setMosfet(PIN_MASTER_MOSFET, master);
  setMosfet(PIN_START_MOSFET, startStop);
  setRelay(PIN_UNLOADER_RELAY, unload);
  setRelay(PIN_IDLE_RELAY, idle);
  setRelay(PIN_KILL_RELAY, kill);
}

void applyOutputs() {
  setMosfet(PIN_FAULT_MOSFET, faultBlinkOutputOn());

  if (fieldManualMode) {
    writeOutputs(manualMasterOn, manualStartStopOn,
                 manualUnloaderOn || emergencyKillActive,
                 manualIdleOn,
                 manualKillOn || emergencyKillActive);
    return;
  }

  bool running = engineRunning();
  bool master = false, unload = false, idle = false, kill = false;
  if (!effectiveAutoOn()) { writeOutputs(false, false, forceUnload, false, false); return; }

  switch (state) {
    case STATE_MASTER_ON_DELAY:
      master = true;
      unload = forceUnload;
      break;
    case STATE_PRECRANK_UNLOAD:
    case STATE_STARTING:
    case STATE_OEM_RUN:
      master = true;
      unload = true;
      break;
    case STATE_RUNNING_LOADED:
      master = true;
      unload = forceUnload;
      idle = forceUnload;   // manual force-unload also requests engine idle
      break;
    case STATE_STOP_UNLOAD:
    case STATE_STOPPING:
      master = true;
      unload = true;
      idle = true;          // 10 s unloaded idle-down, then remain idle through stop
      break;
    case STATE_FAULT:
      master = running && !emergencyKillActive;
      unload = running || forceUnload;
      break;
    default:
      unload = forceUnload;
      break;
  }
  if (emergencyKillActive) { kill = true; unload = true; }
  writeOutputs(master, startStopPulseActive, unload, idle, kill);
}

void enterFieldManualMode() {
  if (!fieldManualMode) {
    fieldManualMode = true;
    startStopPulseActive = false;
    enterState(STATE_WAITING);
    Serial.println(F("MANUAL"));
  }
}

void printStatus() {
  Serial.print(F("STATUS mode=")); Serial.print(fieldManualMode ? F("MAN") : F("AUTO"));
  Serial.print(F(" state=")); Serial.print(stateName(state));
  Serial.print(F(" sw=")); Serial.print(autoOn ? F("ON") : F("OFF"));
  Serial.print(F(" run=")); Serial.print(engineRunning() ? F("Y") : F("N"));
  Serial.print(F(" rpm=")); Serial.print(engineRpm);
  Serial.print(F(" psi=")); Serial.print(tankPressureFiltered, 1);
  Serial.print(F(" V=")); Serial.print(batteryVoltage, 2);
  Serial.print(F(" p=")); Serial.print(pressureCallRaw ? F("CALL") : F("FULL"));
  Serial.print(F(" mon=")); Serial.print(masterMonitorOn ? F("ON") : F("OFF"));
  Serial.print(F(" | M=")); Serial.print(actualMasterOn);
  Serial.print(F(" S=")); Serial.print(actualStartStopOn);
  Serial.print(F(" U=")); Serial.print(actualUnloaderOn);
  Serial.print(F(" I=")); Serial.print(actualIdleOn);
  Serial.print(F(" K=")); Serial.print(actualKillOn);
  Serial.print(F(" pulse=")); Serial.print(startPulseMs);
  Serial.print(F(" fault=")); Serial.print(faultName(faultCode));
  Serial.print(F(" hrs=")); Serial.print(runtimeSeconds / 3600.0, 2);
  Serial.print(F(" cyc=")); Serial.print(startCount);
  Serial.print(F(" rmt=")); Serial.print(remoteAutoPermit ? F("ON") : F("OFF"));
  Serial.print(F(" eff=")); Serial.println(effectiveAutoOn() ? F("ON") : F("OFF"));
}

void setManualOutput(char which, bool on) {
  enterFieldManualMode();
  switch (which) {
    case 'm': manualMasterOn = on; break;
    case 's': manualStartStopOn = on; break;
    case 'u': manualUnloaderOn = on; break;
    case 'i': manualIdleOn = on; break;
    case 'k': manualKillOn = on; break;
  }
  Serial.println(F("OK"));
}

void handleSerialCommand(char* c) {
  for (char* p = c; *p; ++p) *p = (char)tolower(*p);
  while (*c == ' ') c++;
  if (!*c) return;

  if (!strcmp(c, "status")) { printStatus(); return; }
  if (!strcmp(c, "help") || !strcmp(c, "?")) {
    Serial.println(F("master/start/unload/idle/kill on|off; remote auto on|off; set start latch time N; auto; alloff; status"));
    return;
  }
  if (!strcmp(c, "remote auto on")) {
    setRemoteAutoPermit(true);
    Serial.println(F("OK"));
    return;
  }
  if (!strcmp(c, "remote auto off")) {
    setRemoteAutoPermit(false);
    Serial.println(F("OK"));
    return;
  }
  if (!strcmp(c, "auto") || !strcmp(c, "release")) {
    fieldManualMode = false;
    manualMasterOn = manualStartStopOn = manualUnloaderOn = manualIdleOn = manualKillOn = false;
    startStopPulseActive = false;
    enterState(STATE_WAITING);
    Serial.println(F("AUTO"));
    return;
  }
  if (!strcmp(c, "alloff") || !strcmp(c, "all off")) {
    enterFieldManualMode();
    manualMasterOn = manualStartStopOn = manualUnloaderOn = manualIdleOn = manualKillOn = false;
    Serial.println(F("OK"));
    return;
  }

  const char* setp = "set start latch time ";
  if (!strncmp(c, setp, 21)) {
    unsigned long n = strtoul(c + 21, NULL, 10);
    if (n >= 50 && n <= 10000) { startPulseMs = n; Serial.println(F("OK")); }
    else Serial.println(F("ERR 50-10000"));
    return;
  }

  bool on;
  char which = 0;
  char* arg = NULL;
  if (!strncmp(c, "master ", 7)) { which = 'm'; arg = c + 7; }
  else if (!strncmp(c, "start/stop ", 11)) { which = 's'; arg = c + 11; }
  else if (!strncmp(c, "startstop ", 10)) { which = 's'; arg = c + 10; }
  else if (!strncmp(c, "start ", 6)) { which = 's'; arg = c + 6; }
  else if (!strncmp(c, "unload ", 7)) { which = 'u'; arg = c + 7; }
  else if (!strncmp(c, "idle ", 5)) { which = 'i'; arg = c + 5; }
  else if (!strncmp(c, "kill ", 5)) { which = 'k'; arg = c + 5; }

  if (which) {
    if (!strcmp(arg, "on")) on = true;
    else if (!strcmp(arg, "off")) on = false;
    else { Serial.println(F("ERR on/off")); return; }
    setManualOutput(which, on);
    return;
  }
  Serial.println(F("ERR"));
}

void processSerialConsole() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serialCommandBuffer[serialCommandLength] = 0;
      handleSerialCommand(serialCommandBuffer);
      serialCommandLength = 0;
    } else if (serialCommandLength < sizeof(serialCommandBuffer) - 1) {
      serialCommandBuffer[serialCommandLength++] = c;
    } else serialCommandLength = 0;
  }
}

void drawDisplayPage() {
  char line[24];
  char batt[7];
  u8g2.setFont(u8g2_font_5x8_tf);

  if (fieldManualMode) snprintf(line, sizeof(line), "MANUAL H:%lu.%lu", (unsigned long)(runtimeSeconds / 3600UL), (unsigned long)((runtimeSeconds / 360UL) % 10UL));
  else if (faultCode != FAULT_NONE) snprintf(line, sizeof(line), "FLT:%s H:%lu.%lu", faultName(faultCode), (unsigned long)(runtimeSeconds / 3600UL), (unsigned long)((runtimeSeconds / 360UL) % 10UL));
  else {
    const char* autoLabel = !autoOn ? "OFF" : (remoteAutoPermit ? "ON" : "RMT");
    snprintf(line, sizeof(line), "%s A:%s H:%lu.%lu", stateName(state), autoLabel, (unsigned long)(runtimeSeconds / 3600UL), (unsigned long)((runtimeSeconds / 360UL) % 10UL));
  }
  u8g2.drawStr(0, 8, line);

  if (engineRunningByRpm()) snprintf(line, sizeof(line), "RPM %u AVG %u", engineRpm, avgRpm10s);
  else snprintf(line, sizeof(line), "RPM ----");
  u8g2.drawStr(0, 16, line);

  dtostrf(batteryVoltage, 0, 1, batt);
  snprintf(line, sizeof(line), "%u PSI %sV", (unsigned int)(tankPressureFiltered + 0.5), batt);
  u8g2.drawStr(0, 24, line);

  snprintf(line, sizeof(line), "C:%lu M%d S%d U%d", (unsigned long)startCount, actualMasterOn, actualStartStopOn, actualUnloaderOn);
  u8g2.drawStr(0, 32, line);
}

void updateDisplay() {
  if (millis() - lastDisplayMs < DISPLAY_UPDATE_MS) return;
  lastDisplayMs = millis();
  u8g2.firstPage();
  do { drawDisplayPage(); } while (u8g2.nextPage());
}

void setup() {
  digitalWrite(PIN_FAULT_MOSFET, LOW);
  digitalWrite(PIN_MASTER_MOSFET, LOW);
  digitalWrite(PIN_START_MOSFET, LOW);
  digitalWrite(PIN_UNLOADER_RELAY, RELAY_OFF);
  digitalWrite(PIN_IDLE_RELAY, RELAY_OFF);
  digitalWrite(PIN_KILL_RELAY, RELAY_OFF);

  pinMode(PIN_FAULT_MOSFET, OUTPUT);
  pinMode(PIN_MASTER_MOSFET, OUTPUT);
  pinMode(PIN_START_MOSFET, OUTPUT);
  pinMode(PIN_UNLOADER_RELAY, OUTPUT);
  pinMode(PIN_IDLE_RELAY, OUTPUT);
  pinMode(PIN_KILL_RELAY, OUTPUT);
  pinMode(PIN_PRESSURE_SWITCH, INPUT_PULLUP);
  pinMode(PIN_MASTER_MONITOR, INPUT_PULLUP);
  pinMode(PIN_AUTO_SWITCH, INPUT_PULLUP);
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BATT_SENSE, INPUT);
  pinMode(PIN_FORCE_UNLOAD, INPUT);
  pinMode(PIN_TANK_PRESSURE, INPUT);

  Serial.begin(115200);
  delay(300);
  Wire.begin();
  u8g2.begin();
  loadFram();

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  for (uint8_t i = 0; i < RPM_AVG_SAMPLE_COUNT; i++) rpmSamples[i] = 0;
  stateEnteredMs = millis();
  lastRuntimeTickMs = millis();
  lastFramSaveMs = millis();
  Serial.println(F("NorthStar READY - help"));
}

void loop() {
  processSerialConsole();
  updateStartStopPulse();
  readCanRpm();
  readInputs();
  readBatteryVoltage();
  readTankPressure();
  updateChargeRunConfirm();
  updateRpmAverage();
  updateRuntimeCounter();
  if (!fieldManualMode) updateStateMachine();
  applyOutputs();
  updateDisplay();
  if (millis() - lastSerialMs >= SERIAL_STATUS_MS) { lastSerialMs = millis(); printStatus(); }
}
