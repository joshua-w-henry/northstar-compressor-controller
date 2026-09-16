# Wiring Notes

## Master and Start/Stop MOSFET / relay channels

For both D8 Master and D9 Start/Stop:

```text
Nano pin -> 220 ohm -> MOSFET gate
MOSFET gate -> 10k -> MOSFET source
MOSFET source -> control-box ground bus
MOSFET drain -> remote automotive relay terminal 85
Fused +12 V from control box -> remote automotive relay terminal 86
```

At each remote relay, install a flyback diode directly across the relay coil:

```text
stripe / cathode -> terminal 86 / +12 V
plain / anode    -> terminal 85 / MOSFET drain
```

Relay contacts:

```text
30 -> OEM switch wire A
87 -> OEM switch wire B
87a unused
```

Recommended harness between control box and each remote relay:
- two wires per relay
- fused +12 V feed
- MOSFET-drain switched-ground return
- 18-20 AWG recommended for ruggedness

## Individual relay modules (A3 / A1 / A2)

The three small relay modules used for:
- A3 unloader
- A1 idle
- A2 kill

are configured with their trigger-selection jumpers set for **LOW-level trigger**.

Firmware convention:

```text
Nano output LOW  = relay energized / command ON
Nano output HIGH = relay released / command OFF
```

Keep all three modules in the same LOW-trigger jumper position so the hardware matches the firmware assumptions.

## Unloader solenoid

Relay module contact wiring:

```text
fused +12 V -> relay NO
relay COM -> solenoid red
solenoid black -> ground
NC unused
```

With the relay released, the solenoid red lead should be near 0 V. With the relay energized, COM connects to NO and the solenoid red lead should be near +12 V.

The solenoid flyback diode goes across the solenoid coil, not inline:

```text
stripe / cathode -> solenoid red / switched +12 V
plain / anode    -> solenoid black / ground
```

## Grounding

- MOSFET source legs may connect to the main control-box ground bus.
- Gate pulldown resistors should connect locally from gate to MOSFET source.
- Avoid carrying starter current through the control-box ground path.
- The control ground should reference the same battery / engine negative system.

## Power conditioning

Current control box already has:
- electrolytic capacitor on the 5 V rail
- ceramic capacitor on the 5 V rail

Planned addition:
- 470-1000 uF electrolytic close to the buck converter 12 V input
- 25 V minimum rating; 35 V preferred

## Battery note

The original clean bench operation used an Optima RedTop. Later tests used a small 20 Ah battery and showed intermittent poor cranking and control instability. A cranking-capable battery is a primary test variable for the next validation cycle.
