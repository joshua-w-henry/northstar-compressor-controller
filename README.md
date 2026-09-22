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

The ESP32 sidecar provides monitoring/logging plus one deliberately narrow
control request:
- MQTT / Home Assistant telemetry
- pressure history and leak-down monitoring
- runtime and cycle history
- SD black-box logging
- read-only SD log pull
- remote AUTO permit ON/OFF

The Nano remains the real-time controller. The ESP32 cannot directly command an
engine start or drive Master, Start/Stop, Unloader, Idle, Kill, or Reset through
the Home Assistant control path. Remote AUTO is only an additional permission
gate: the physical AUTO/OFF switch and the Nano state machine remain authoritative.

## Build

This is a PlatformIO project targeting an Arduino Nano ATmega328P.

See `platformio.ini` for dependencies and serial settings.
