# Power Architecture and Reusable 12 V Load Strategy

## Compressor controller direction

Adopt the **Infineon BTS50015-1TAD** as the preferred smart high-side switch for independently switched 12 V power branches.

The device replaces the common discrete stack of:
- logic-level MOSFET / transistor driver
- 12 V automotive relay
- separate branch-current sensor
- much of the discrete load-protection circuitry

Each independently controlled 12 V branch may use one BTS50015-1TAD. Group loads into sensible branches rather than assigning one device to every small component.

Planned compressor architecture:

```text
12 V AGM battery
 |
 +-- fused always-on buck --> ESP32-S3
 |
 +-- fused smart high-side branch(es) --> switched 12 V loads
        |
        +-- controller / Nano / CAN / sensors as appropriate
        +-- engine ECU / EFI power as appropriate after harness mapping
        +-- 12 V actuators / accessories as appropriate
```

The ESP32 remains the always-on power supervisor. Switched branches default OFF unless deliberately enabled. Hardware input biasing must make a floating/reset GPIO fail to OFF.

Retain conventional fusing near the battery. The smart high-side switch does not replace branch wiring protection.

Use the BTS diagnostic/current-sense output where practical so the controller can detect:
- commanded ON with little/no current
- abnormal overcurrent
- load or wiring faults
- branch-current trends

## Dry-contact outputs

The BTS50015-1TAD is **not** a dry-contact replacement.

Floating OEM button/switch interfaces such as Start/Stop, Master, Idle, or other isolated contact closures should use:
- PhotoMOS / solid-state relay where voltage/current limits fit, or
- a conventional relay where higher current, unusual voltage, or true mechanical isolation is required.

The final PCB should therefore distinguish between:
1. smart high-side 12 V power outputs, and
2. isolated dry-contact outputs.

## Reusable ECU / EFI design standard

Treat the BTS50015-1TAD, or a later equivalent automotive smart high-side switch, as the **default starting point for future ECU/EFI 12 V load-distribution channels**.

Candidate future uses include:
- ECU / controller power rails
- fuel-pump feeds
- ignition-coil power feeds
- oxygen-sensor heater power
- fans and pumps
- actuator / solenoid power rails
- auxiliary 12 V branches

Do not use this device in place of dedicated injector or ignition drivers where fast current shaping, peak-and-hold control, or other specialized drive behavior is required.

The design goal for future ECU work is protected, diagnosable power distribution: the controller should know both what it commanded and what electrical load actually occurred.

## Stock / prototyping plan

Likely bench stock:
- **25–50 BTS50015-1TAD devices**
- several package-to-pin / pad-to-prong test adapters for breadboard and bench evaluation
- suitable sense resistors and protection/passive components for current-sense characterization

Before final PCB freeze, validate:
- logic-level operation from the ESP32 / MCU
- current-sense scaling across representative loads
- thermal behavior at expected branch current
- inductive-load behavior where applicable
- OFF-state leakage / parked battery draw
- fault behavior and recovery
