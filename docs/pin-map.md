# Pin Map

## Inputs

| Nano pin | Function | Active state / notes |
|---|---|---|
| D2 | Pressure switch opto | LOW = CALL for air, HIGH = FULL |
| D3 | OEM master monitor opto | LOW = master ON |
| D4 | AUTO/OFF opto | LOW = AUTO, HIGH = OFF |
| D5 | Reset / fault-clear button | LOW = pressed |
| A0 | Battery-voltage divider | 100k / 33k divider, calibrated in firmware |
| A6 | Force-unload opto | Analog-only; <600 = active |
| A7 | 200 PSI pressure transducer | Assumed 0.5-4.5 V = 0-200 PSI |

## Outputs — Phase 0 hardware rebuild

| Nano pin | Function | Output hardware | Logic |
|---|---|---|---|
| D8 | Master switch bypass | N-MOSFET -> remote 12 V automotive relay | Active HIGH |
| D9 | Start/Stop command -> MOSFET -> remote 12 V automotive relay | Active HIGH |
| D6 | Fault lamp | N-MOSFET | Active HIGH |
| A3 | Unloader solenoid | Individual relay module | Active LOW relay input |
| A1 | Idle dry contact | Individual relay module | Active LOW relay input |
| A2 | Kill dry contact | Individual relay module | Active LOW relay input |

## CAN

| Nano pin | MCP2515 |
|---|---|
| D10 | CS |
| D11 | MOSI / SI |
| D12 | MISO / SO |
| D13 | SCK |

CAN configuration:
- 500 kbps
- MCP2515 8 MHz oscillator
- RPM extended ID: `0x0C665500`
- RPM bytes: `data[2]`, `data[3]`

## I2C

| Nano pin | Function |
|---|---|
| A4 | SDA |
| A5 | SCL |

Current I2C devices:
- 0.91-inch 128x32 OLED
- FRAM at address `0x50`

## ESP32 sidecar interface

Telemetry path:
- Nano D1 / TX (5 V) -> existing level divider -> ESP32 RX
- Common ground

Limited return-control path:
- ESP32 GPIO 8 / TX (3.3 V) -> 220 Ω series resistor -> removable jumper -> Nano D0 / RX
- 3.3 V from the ESP32 is valid HIGH logic for the ATmega328P Nano input.
- Remove the jumper for Nano programming/serial troubleshooting if needed.
- The 220 Ω value is field-proven. A 1 kΩ series resistor did not pull Nano RX low enough because the Nano onboard USB-serial interface also biases RX0 through its own resistance; the measured RX idle node was about 4.2 V with 1 kΩ. Replacing it with 220 Ω restored reliable UART control.

The return path accepts only the narrow serial command surface implemented by the
Nano. The Home Assistant-facing control is `remote auto on|off`; it changes the
remote AUTO permit but does not directly command Master, Start/Stop, Unloader,
Idle, Kill, or Reset.

AUTO authority is AND-gated:

```text
effective AUTO = physical AUTO switch AND remote AUTO permit
```

The physical OFF position always wins. A deliberate local OFF -> AUTO switch
cycle clears a persisted remote inhibit, so local recovery does not depend on
the ESP32, MQTT, or Home Assistant.
