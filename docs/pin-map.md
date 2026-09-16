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

## Outputs — current rebuild plan

| Nano pin | Function | Output hardware | Logic |
|---|---|---|---|
| D8 | Master switch bypass | N-MOSFET -> 12 V automotive relay | Active HIGH |
| D9 | Start/Stop button bypass | N-MOSFET -> 12 V automotive relay | Active HIGH |
| D6 | Fault lamp | N-MOSFET | Active HIGH; planned change from baseline code |
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

## Planned ESP32 sidecar interface

Initial monitoring-only link:
- Nano D1 / TX -> level divider -> ESP32 RX
- Common ground

Future optional return path:
- ESP32 TX -> Nano D0 / RX through a removable jumper
