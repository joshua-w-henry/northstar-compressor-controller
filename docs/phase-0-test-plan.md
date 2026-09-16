# Phase 0 Test Plan

Goal: reliable local compressor control without depending on ESP32, Wi-Fi, Home Assistant, or a laptop.

## 1. Power and safe boot

Verify before reconnecting OEM control contacts:
- control-box +12 V feed is fused
- control-box ground is solid
- buck converter input capacitor is installed
- Nano 5 V electrolytic and ceramic capacitors remain in place
- D8/D9/D6 MOSFETs default OFF at boot
- no relay clicks unexpectedly during boot

## 2. Output-channel bench test

Test one channel at a time with OEM switch wires disconnected.

Expected:
- D8 HIGH -> only Master automotive relay energizes
- D8 LOW -> Master relay releases
- D9 HIGH -> only Start/Stop automotive relay energizes
- D9 LOW -> Start/Stop relay releases
- D6 HIGH -> fault lamp turns on
- D6 LOW -> fault lamp turns off
- A3 -> unloader relay operates correctly
- A1 -> idle relay closes/opens its dry contact correctly
- A2 -> kill relay closes/opens its dry contact correctly

Confirm every automotive relay uses contacts 30/87; 87a is unused.

## 3. Input validation

With Serial status visible, verify each input independently:
- D2 pressure switch: CALL / FULL
- D3 master monitor: ON / OFF
- D4 AUTO/OFF: AUTO / OFF
- D5 reset: pressed / released
- A6 force unload: active / inactive
- A0 battery voltage is plausible
- A7 tank pressure matches mechanical gauge closely enough for testing

## 4. CAN validation

Verify:
- MCP2515 initializes at 500 kbps / 8 MHz
- CAN frame count increases
- RPM frame count increases while engine is running
- displayed RPM agrees with observed engine speed

Known RPM frame:
- extended ID `0x0C665500`
- RPM = `(data[2] << 8) | data[3]`
- reject instantaneous RPM above 4000

## 5. Start sequence test

Target sequence:

```text
WAIT
-> pressure CALL confirmed
-> Master ON
-> 4 s delay
-> confirm master monitor
-> unloader ON
-> 1 s delay
-> Start/Stop pulse 750 ms
-> detect running by CAN RPM >= 400 or charging-voltage fallback
-> run unloaded 15 s
-> unloader OFF
-> RUN loaded
```

## 6. Stop sequence test

After start/run reliability is proven, enable pressure auto-stop and verify:

```text
pressure FULL stable 3 s
-> unloader ON
-> wait 8 s
-> Start/Stop pulse 750 ms
-> hold Master ON 6 s after pulse ends
-> Master OFF
-> WAIT
```

AUTO/OFF behavior remains special: switching AUTO to OFF releases controller outputs rather than issuing a stop or kill command.

## 7. Fault validation

Validate fault indication and safe response for:
- master monitor failure
- start failure
- engine-lost condition
- overspeed logic

Kill relay is reserved for overspeed safety, not normal stopping.

## Phase 0 completion criteria

- no false triggers at boot
- repeatable cold and warm starts
- stable CAN and display during cranking
- correct unloaded warmup and loading
- correct automatic stop sequence
- manual AUTO/OFF release behaves as designed
- no relay chatter or brownout-related resets
- wiring and firmware documentation match the installed hardware
