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

Future power-distribution direction:
- Infineon BTS50015-1TAD smart high-side switches for independently controlled 12 V power branches
- PhotoMOS / relays retained for true floating dry-contact interfaces
- ESP32 remains the always-on power supervisor; most other loads may move to switched 12 V branches
- See `docs/power-architecture.md` for the compressor plan and the reusable ECU/EFI 12 V load strategy

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


## Backlog priorities

- **P1 — Cold-start EFI priming: initial field tests successful; formal validation still open.** Nano firmware was successfully built and uploaded via USB; warm-start serial capture confirmed fresh ECU block temperature 36.4°C / 97.5°F, one prime, 350 ms Start/Stop pulse, first-crank start, and cycle count 163 -> 164. A subsequent cold-start operator observation confirmed **three priming cycles and a successful first-crank start**, but **no cold-start serial log was captured**. Complete cold-start event/timing capture, actual fuel-pump re-prime verification and interlock tests before merging. Firmware implementation and test plan are in in [draft PR #1](https://github.com/joshua-w-henry/northstar-compressor-controller/pull/1), branch `feature/cold-start-three-prime-can-temp`. Build with PlatformIO and upload **directly to the Nano via its USB/wired programming connection**, not over the ESP32. Bench-test before reinstalling or enabling unattended AUTO. Engine-block temperature is decoded on the Nano directly from ECU CAN: below 75°F (or missing/stale temperature) calls for **three Master ON priming intervals** (4 s ON, 1 s OFF, 3 s ON, 1 s OFF, 3 s ON), followed by the normal 1 s precrank unload and one bounded 350 ms Start/Stop command. Warm engine: original one-prime startup. Verify actual fuel-pump behavior, no spurious starts during cancellation, and **one cycle count per requested start, no Hobbs time during priming**. See [cold-start priming validation](https://github.com/joshua-w-henry/northstar-compressor-controller/blob/feature/cold-start-three-prime-can-temp/docs/cold-start-priming.md) in the PR branch. **Status: wired deployment and first warm/cold starts observed, but full acceptance testing still pending.**
- **P3 — ESP32-to-Nano OTA programming (deferred optimization, not required for priming).** Keep Nano and ESP32 firmware independently maintainable. Later implement secure ESP32 Wi-Fi-to-UART AVR bootloader programming using the **existing bidirectional UART** (Nano D1/TX -> divider -> ESP32 GPIO18/RX; ESP32 GPIO8/TX -> 220 Ω -> Nano D0/RX). Add **one new ESP32 GPIO through a transistor pull-down to Nano hardware RST**, including appropriate biasing to prevent accidental reset at boot. **Nano D5 momentary button remains separate: it clears firmware fault state, not hardware RST.** Require verified stopped engine, AUTO/start inhibit and safe outputs during the upload, firmware integrity/write verification, plus local wired recovery if OTA fails. Check the installed Nano bootloader/baud and ATmega328P flash capacity before implementation. Implement OTA only after cold-start priming is stable through wired uploads.
- **P3 — Automatic receiver drain:** add a normally-closed, pressure-rated 12 V drain solenoid at the receiver low point with manual isolation/service capability. Preferred behavior is a brief timed drain after normal compressor shutdown, with a hard maximum-open timeout and optional manual/HA "drain now" command. Candidate final drive method: dedicated BTS50015-1TAD smart high-side channel with current/fault feedback.
