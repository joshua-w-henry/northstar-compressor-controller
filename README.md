# NorthStar Compressor Controller

Arduino Nano / PlatformIO controller for the NorthStar gas-engine air compressor automation project.

## Current phase

Phase 0: reliable local hardwired control.

The immediate goal is to make the machine start, run unloaded, load, stop, and fault predictably without depending on networking or remote services.

## Current baseline

This repository begins with the last-known Phase 0 prototype firmware before the September 2026 control-box rebuild.

Known-good subsystems include:
- Mechanical compressor conversion
- Manual load/unload operation
- 200 PSI pressure transducer
- Battery-voltage sensing
- 0.91-inch OLED
- FRAM counters
- MCP2515 CAN interface after module replacement
- Engine RPM decode from extended CAN ID `0x0C665500`

RPM decode:

```text
RPM = (data[2] << 8) | data[3]
```

## Current hardware direction

Critical outputs:
- D8: Master command -> MOSFET -> remote 12 V automotive relay -> OEM master-switch dry contact
- D9: Start/Stop command -> MOSFET -> remote 12 V automotive relay -> OEM start/stop dry contact
- D6: Fault lamp -> MOSFET output (planned hardware rebuild change)

Mechanical relay outputs retained:
- A3: Unloader solenoid relay
- A1: Idle dry-contact relay
- A2: Kill dry-contact relay

## Phase 1 plan

After Phase 0 is reliable, add an ESP32 sidecar for monitoring and logging, including:
- MQTT / Home Assistant
- pressure history and leak-down monitoring
- runtime and cycle history
- larger local display
- later, carefully limited safety/control requests

The Nano remains the real-time controller; the ESP32 is initially an observer/logger.

## Build

This is a PlatformIO project targeting an Arduino Nano ATmega328P.

See `platformio.ini` for dependencies and serial settings.
