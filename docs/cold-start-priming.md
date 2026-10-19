# Temperature-dependent three-prime cold start (bench validation required)

## Purpose

Improve NorthStar e420 EFI cold starting by allowing the OEM fuel pump to prime
three times before the controller sends a **single** OEM Start/Stop dry-contact
command. Do not disturb the production OEM-managed start retry mechanism.

This enhancement is isolated to the Arduino Nano controller. The ESP32 and
Home Assistant connections are not required for automatic priming.

## Temperature inputs

- Engine-block temperature: extended CAN ID `0x0C66550F`, bytes D4-D5.
- Intake air temperature: extended CAN ID `0x0C665509`, bytes D4-D5.
- Both decode: unsigned big-endian raw minus 400 = temperature in **deci-°C**.
- Only a **fresh engine-block** reading determines warm versus cold.
- Freshness window: 1500 ms at the end of the first Master ON period.
- Below **75°F** (less than 23.9°C at the ECU's 0.1°C resolution) means cold.
- If block temperature is absent, stale, or invalid, conservatively use
  three primes. IAT is decoded for diagnostic/future use, **not** used to
  override the block-temperature decision.

## Cold startup timeline

The first Master ON period is the existing 4-second settling delay.

| Step | State | Master | Duration | Start/Stop output |
|---|---|---|---|---|
| 1 | M-ON | ON | 4 s | OFF |
| 2 | P1OFF | OFF | 1 s | OFF |
| 3 | P2ON | ON | 3 s | OFF |
| 4 | P2OFF | OFF | 1 s | OFF |
| 5 | P3ON | ON | 3 s | OFF |
| 6 | UNLD | ON | 1 s | OFF |
| 7 | STRT | ON | OEM-managed | 350 ms max pulse |

Warm startup follows the original M-ON (4 s) -> UNLD (1 s) -> STRT
(350 ms max pulse) path. The extra priming adds 8 seconds on a cold start.
Master remains ON throughout OEM start attempts, including the pre-existing
second supervised OEM Start/Stop attempt after its 70-second window.

The 1-second Master OFF period is a configurable starting value, not yet
verified as sufficient to reset the OEM ECU fuel-pump priming logic.

## Interlocks / outputs

- Physical AUTO/OFF and the existing remote AUTO permission remain dominant.
- If pressure demand disappears, or force unload activates, prestart states
  return to WAIT without energizing the Start/Stop output.
- If engine RPM indicates already running during prestart, enter OEM_RUN.
- Master OFF periods switch **only the engine's Master output**. The Nano,
  5 V supply, FRAM, CAN processing, and safety supervision remain energized.
- Do not allow the Master switching states to issue Start/Stop pulses.
- Preserve the hard 350 ms Start/Stop output limit and existing OEM retries.

## Counting / Hobbs

- `startCount` increments **once** at the first Start/Stop command in
  `STATE_PRECRANK_UNLOAD`, not when Master toggles.
- OEM-managed retries under `STATE_STARTING` do not increment `startCount`.
- `runtimeSeconds` advances only when the running detector is true and the
  controller is **not in unconfirmed `STATE_STARTING`**. This avoids assigning
  Hobbs runtime to long 400-500 RPM failed catches.
- Manual running and shutdown phases continue to accrue actual run time.

## Bench acceptance checklist (do not flash to a running compressor untested)

1. With outputs disconnected from engine, simulate pressure CALL at
   block temperature 20°C: observe three Master ON edges, two OFF periods
   (1 second each), no Start/Stop output until after the third prime,
   then exactly one bounded 350 ms pulse.
2. Simulate 30°C: observe one Master ON edge, then normal startup.
3. Test 23.8°C (cold) and 23.9°C (warm) to verify the exact threshold.
4. Omit the engine-block CAN frame: verify fallback to three primes.
5. Send an old/stale frame only: verify the same three-prime fallback.
6. Cancel AUTO / remote permission / pressure call during each prime
   state: all engine commands deenergize safely, no starter pulse.
7. Simulate a 400-500 RPM failed ignition catch for several seconds in
   STARTING: Hobbs must not advance. On confirmed 800+ RPM start, Hobbs
   resumes normally.
8. After three primes and a normal start: FRAM cycle count increases by
   exactly one; test the OEM second Start/Stop attempt without an
   additional cycle count.
9. Test Master monitor behavior, relay release time, and actual EFI
   fuel pump response on the engine before enabling unattended AUTO.
10. After bench tests, field-check Master and Start/Stop timing, and
    record cold/warm start outcomes and battery voltage.

## Console and telemetry

- `EV PRIME block_temp_dC=<value|UNKNOWN> count=<1|3>`
- `EV ENTER P1OFF`, `P2ON`, `P2OFF`, `P3ON` appear during cold priming.
- `EV PRIME CANCEL demand` marks a prestart demand/force-unload abort.
- ESP32 already forwards Nano `EV` lines to MQTT event telemetry.
