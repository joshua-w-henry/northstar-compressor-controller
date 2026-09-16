#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// ================================================================
// NorthStar Compressor Controller - BENCH I/O TEST MODE
//
// Purpose:
//   Commission each control-box input and output independently.
//   The normal compressor state machine is NOT compiled in this mode.
//   Every reboot starts with every output OFF.
//
// Serial monitor: 115200 baud, newline ending.
// ================================================================

// ---------------------- Pin map ----------------------

const uint8_t PIN_PRESSURE_SWITCH = 2;   // LOW = CALL, HIGH = FULL
const uint8_t PIN_MASTER_MONITOR  = 3;   // LOW = OEM master ON
const uint8_t PIN_AUTO_SWITCH     = 4;   // LOW = AUTO
const uint8_t PIN_RESET_BUTTON    = 5;   // LOW = pressed

const uint8_t PIN_BATT_SENSE      = A0;
const uint8_t PIN_FORCE_UNLOAD    = A6;  // <600 = active
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
const uint32_t RPM_CAN_ID         = 0x0C665500UL;
const uint16_t MAX_ACCEPTED_RPM   = 4000;
const unsigned long RPM_STALE_MS  = 1500;

// ---------------------- Analog calibration ----------------------

const float ADC_REF_V = 5.0;
const float BATT_DIVIDER_FACTOR = (100.0 + 33.0) / 33.0;
const float BATT_CAL = 1.067;

const float PRESSURE_SENSOR_MIN_V = 0.5;
const float PRESSURE_SENSOR_SPAN_V = 4.0;
const float PRESSURE_SENSOR_MAX_PSI = 200.0;

// ---------------------- Hardware objects ----------------------

MCP2515 mcp2515(PIN_CAN_CS);

// ---------------------- Bench output states ----------------------

bool outMaster = false;
bool outStart = false;
bool outFault = false;
bool outUnloader = false;
bool outIdle = false;
bool outKill = false;

// ---------------------- Live measurements ----------------------

uint16_t engineRpm = 0;
unsigned long lastRpmFrameMs = 0;
uint32_t canAnyFrameCount = 0;
uint32_t canRpmFrameCount = 0;
uint32_t canRejectedRpmCount = 0;

float batteryVoltage = 0.0;
float tankPressurePsi = 0.0;
int forceUnloadRaw = 1023;

unsigned long lastStatusMs = 0;

// ---------------------- Serial command buffer ----------------------

char commandBuffer[40];
uint8_t commandLength = 0;

// ================================================================
// Output helpers
// ================================================================

void setRelayModule(uint8_t pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

void setMosfet(uint8_t pin, bool on) {
  digitalWrite(pin, on ? HIGH : LOW);
}

void applyBenchOutputs() {
  setMosfet(PIN_MASTER_MOSFET, outMaster);
  setMosfet(PIN_START_MOSFET, outStart);
  setMosfet(PIN_FAULT_MOSFET, outFault);

  setRelayModule(PIN_UNLOADER_RELAY, outUnloader);
  setRelayModule(PIN_IDLE_RELAY, outIdle);
  setRelayModule(PIN_KILL_RELAY, outKill);
}

void allOutputsOff() {
  outMaster = false;
  outStart = false;
  outFault = false;
  outUnloader = false;
  outIdle = false;
  outKill = false;
  applyBenchOutputs();
}

// ================================================================
// Inputs / measurements
// ================================================================

uint16_t analogReadAveraged(uint8_t pin, uint8_t samples = 16) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(250);
  }
  return (uint16_t)(sum / samples);
}

void readAnalogInputs() {
  uint16_t battRaw = analogReadAveraged(PIN_BATT_SENSE);
  float battAdcV = ((float)battRaw / 1023.0) * ADC_REF_V;
  batteryVoltage = battAdcV * BATT_DIVIDER_FACTOR * BATT_CAL;

  uint16_t pressureRaw = analogReadAveraged(PIN_TANK_PRESSURE);
  float pressureV = ((float)pressureRaw / 1023.0) * ADC_REF_V;
  float psi = (pressureV - PRESSURE_SENSOR_MIN_V) *
              (PRESSURE_SENSOR_MAX_PSI / PRESSURE_SENSOR_SPAN_V);
  if (psi < 0.0) psi = 0.0;
  if (psi > PRESSURE_SENSOR_MAX_PSI) psi = PRESSURE_SENSOR_MAX_PSI;
  tankPressurePsi = psi;

  forceUnloadRaw = analogRead(PIN_FORCE_UNLOAD);
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
      }
    }
  }

  if ((millis() - lastRpmFrameMs) > RPM_STALE_MS) {
    engineRpm = 0;
  }
}

// ================================================================
// Serial reporting
// ================================================================

void printHelp() {
  Serial.println(F("COMMANDS:"));
  Serial.println(F("  master on | master off"));
  Serial.println(F("  start on | start off"));
  Serial.println(F("  fault on | fault off"));
  Serial.println(F("  unloader on | unloader off"));
  Serial.println(F("  idle on | idle off"));
  Serial.println(F("  kill on | kill off"));
  Serial.println(F("  alloff"));
  Serial.println(F("  inputs"));
  Serial.println(F("  help"));
}

void printStatus() {
  bool pressureCall = (digitalRead(PIN_PRESSURE_SWITCH) == LOW);
  bool masterMonitor = (digitalRead(PIN_MASTER_MONITOR) == LOW);
  bool autoOn = (digitalRead(PIN_AUTO_SWITCH) == LOW);
  bool resetPressed = (digitalRead(PIN_RESET_BUTTON) == LOW);
  bool forceUnload = (forceUnloadRaw < 600);

  Serial.println(F("--- BENCH I/O ---"));
  Serial.print(F("IN  D2 PRESSURE=")); Serial.println(pressureCall ? F("CALL") : F("FULL"));
  Serial.print(F("IN  D3 MASTERMON=")); Serial.println(masterMonitor ? F("ON") : F("OFF"));
  Serial.print(F("IN  D4 AUTO=")); Serial.println(autoOn ? F("ON") : F("OFF"));
  Serial.print(F("IN  D5 RESET=")); Serial.println(resetPressed ? F("PRESSED") : F("OPEN"));
  Serial.print(F("IN  A6 FORCE_UNLD=")); Serial.print(forceUnload ? F("ON") : F("OFF"));
  Serial.print(F(" RAW=")); Serial.println(forceUnloadRaw);
  Serial.print(F("IN  A0 BATT=")); Serial.print(batteryVoltage, 2); Serial.println(F(" V"));
  Serial.print(F("IN  A7 PRESSURE=")); Serial.print(tankPressurePsi, 1); Serial.println(F(" PSI"));
  Serial.print(F("IN  CAN RPM=")); Serial.print(engineRpm);
  Serial.print(F(" ANY=")); Serial.print(canAnyFrameCount);
  Serial.print(F(" RPMFRAMES=")); Serial.print(canRpmFrameCount);
  Serial.print(F(" REJECT=")); Serial.println(canRejectedRpmCount);

  Serial.print(F("OUT D8 MASTER=")); Serial.println(outMaster ? F("ON") : F("OFF"));
  Serial.print(F("OUT D9 START=")); Serial.println(outStart ? F("ON") : F("OFF"));
  Serial.print(F("OUT D6 FAULT=")); Serial.println(outFault ? F("ON") : F("OFF"));
  Serial.print(F("OUT A3 UNLOADER=")); Serial.println(outUnloader ? F("ON") : F("OFF"));
  Serial.print(F("OUT A1 IDLE=")); Serial.println(outIdle ? F("ON") : F("OFF"));
  Serial.print(F("OUT A2 KILL=")); Serial.println(outKill ? F("ON") : F("OFF"));
  Serial.println();
}

// ================================================================
// Serial commands
// ================================================================

void setOutputCommand(const char* name, bool value) {
  if (strcmp(name, "master") == 0) outMaster = value;
  else if (strcmp(name, "start") == 0) outStart = value;
  else if (strcmp(name, "fault") == 0) outFault = value;
  else if (strcmp(name, "unloader") == 0) outUnloader = value;
  else if (strcmp(name, "idle") == 0) outIdle = value;
  else if (strcmp(name, "kill") == 0) outKill = value;
  else {
    Serial.println(F("ERR unknown output"));
    return;
  }

  applyBenchOutputs();
  Serial.print(F("OK "));
  Serial.print(name);
  Serial.println(value ? F(" ON") : F(" OFF"));
}

void processCommand(char* cmd) {
  while (*cmd == ' ') cmd++;
  if (*cmd == '\0') return;

  for (char* p = cmd; *p; ++p) {
    if (*p >= 'A' && *p <= 'Z') *p = *p - 'A' + 'a';
  }

  if (strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(cmd, "inputs") == 0 || strcmp(cmd, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(cmd, "alloff") == 0) {
    allOutputsOff();
    Serial.println(F("OK ALL OUTPUTS OFF"));
    return;
  }

  char* space = strchr(cmd, ' ');
  if (!space) {
    Serial.println(F("ERR use: <output> on|off, or help"));
    return;
  }

  *space = '\0';
  char* value = space + 1;
  while (*value == ' ') value++;

  if (strcmp(value, "on") == 0) {
    setOutputCommand(cmd, true);
  } else if (strcmp(value, "off") == 0) {
    setOutputCommand(cmd, false);
  } else {
    Serial.println(F("ERR value must be on or off"));
  }
}

void serviceSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      processCommand(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    }
  }
}

// ================================================================
// Setup / loop
// ================================================================

void setup() {
  // Establish safe output levels before enabling output mode.
  digitalWrite(PIN_FAULT_MOSFET, LOW);
  digitalWrite(PIN_MASTER_MOSFET, LOW);
  digitalWrite(PIN_START_MOSFET, LOW);
  digitalWrite(PIN_UNLOADER_RELAY, HIGH);
  digitalWrite(PIN_IDLE_RELAY, HIGH);
  digitalWrite(PIN_KILL_RELAY, HIGH);

  pinMode(PIN_FAULT_MOSFET, OUTPUT);
  pinMode(PIN_MASTER_MOSFET, OUTPUT);
  pinMode(PIN_START_MOSFET, OUTPUT);
  pinMode(PIN_UNLOADER_RELAY, OUTPUT);
  pinMode(PIN_IDLE_RELAY, OUTPUT);
  pinMode(PIN_KILL_RELAY, OUTPUT);

  allOutputsOff();

  pinMode(PIN_PRESSURE_SWITCH, INPUT_PULLUP);
  pinMode(PIN_MASTER_MONITOR, INPUT_PULLUP);
  pinMode(PIN_AUTO_SWITCH, INPUT_PULLUP);
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BATT_SENSE, INPUT);
  pinMode(PIN_FORCE_UNLOAD, INPUT);
  pinMode(PIN_TANK_PRESSURE, INPUT);

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("***************************************"));
  Serial.println(F("*** NORTHSTAR - BENCH I/O TEST MODE ***"));
  Serial.println(F("*** NORMAL STATE MACHINE IS DISABLED ***"));
  Serial.println(F("***************************************"));

  SPI.begin();
  mcp2515.reset();
  MCP2515::ERROR canSpeedResult = mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  if (canSpeedResult == MCP2515::ERROR_OK) Serial.println(F("CAN: bitrate OK"));
  else Serial.println(F("CAN: bitrate FAIL"));
  mcp2515.setNormalMode();

  Serial.println(F("All outputs forced OFF."));
  printHelp();
  Serial.println();
}

void loop() {
  serviceSerial();
  readCanRpm();
  readAnalogInputs();

  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    printStatus();
  }
}
