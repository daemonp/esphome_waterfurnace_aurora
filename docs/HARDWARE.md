# Hardware Setup & Wiring

## Requirements

1. **ESP32 or ESP8266 Development Board** (e.g., Wemos D1 Mini, NodeMCU, ESP32-DevKit)
2. **TTL to RS485 Adapter** with manual DE/RE flow control pins (see below)
3. **RJ45 Connector/Cable** to connect to the AID Tool port on the heat pump

## Recommended RS485 Module

Use a MAX485 module with **exposed DE/RE pins** for manual flow control. A known working module is the [Alinan MAX485 RS485 Transceiver Module](https://www.amazon.com/dp/B00NIOLNAG).

> **Important**: Avoid RS485 modules with "automatic flow control" that only have VCC/TXD/RXD/GND pins. These lack the DE/RE pins needed for reliable Modbus RTU timing and will cause communication errors.

## AID Tool Port (RJ45) Pinout

The AID Tool port on the front of your heat pump uses an RJ45 jack. The 8 pins carry **two completely different buses** — RS-485 data and 24VAC thermostat power:

```
        RJ45 Jack (looking at the port on the heat pump)
    ┌─────────────────────────────┐
    │  8  7  6  5  4  3  2  1    │
    │  ┃  ┃  ┃  ┃  ┃  ┃  ┃  ┃   │
    └──╂──╂──╂──╂──╂──╂──╂──╂───┘
       │  │  │  │  │  │  │  │
       │  │  │  │  │  │  │  └── Pin 1: RS-485 A+ (data)
       │  │  │  │  │  │  └───── Pin 2: RS-485 B- (data)
       │  │  │  │  │  └──────── Pin 3: RS-485 A+ (data)
       │  │  │  │  └─────────── Pin 4: RS-485 B- (data)
       │  │  │  └────────────── Pin 5: R  - 24VAC ⚡ DANGER
       │  │  └───────────────── Pin 6: C  - 24VAC ⚡ DANGER
       │  └──────────────────── Pin 7: R  - 24VAC ⚡ DANGER
       └─────────────────────── Pin 8: C  - 24VAC ⚡ DANGER
```

```
    ┌───────────────────────────────────────────────────────┐
    │  Signal    Pins     Wire Color (T568B)    Use         │
    ├───────────────────────────────────────────────────────┤
    │  A+  ───  1, 3     White/Orange,          RS-485 data │
    │                     White/Green            (connect)   │
    │  B-  ───  2, 4     Orange, Blue           RS-485 data │
    │                                            (connect)   │
    │  R   ───  5, 7     White/Blue,            24VAC power │
    │                     White/Brown            DO NOT USE  │
    │  C   ───  6, 8     Green, Brown           24VAC power │
    │                                            DO NOT USE  │
    └───────────────────────────────────────────────────────┘
```

> ### ⚠️ DANGER: 24VAC Power Pins (5, 6, 7, 8)
>
> Pins 5-8 carry **24VAC thermostat bus power** (C and R lines). These are **NOT** data pins.
>
> **DO NOT connect these pins to anything** — not to your RS-485 adapter, not to your ESP, not to ground, not to each other. If you short a 24VAC pin against a data pin, a ground wire, or anything else:
>
> - **Best case**: You blow the 3A automotive fuse inside the heat pump (replaceable, but you'll need to find it)
> - **Worst case**: You destroy the ABC (Aurora Base Control) board — a very expensive repair on a $40K+ heat pump
>
> **Only connect pins 1-4** (RS-485 data). Leave pins 5-8 completely unconnected and insulated. If you're cutting an ethernet cable, strip and connect only the white/orange, orange, white/green, and blue wires. Cut the remaining four wires short and tape/shrink-wrap them so they can't accidentally touch anything.

## Wiring

Connect **only the RS-485 data pins** (1-4) from the AID Tool port to your RS485 adapter, then the adapter to your ESP:

| Heat Pump (RJ45) | Wire Color (T568B) | RS485 Adapter | ESP32/8266 |
| :--- | :--- | :--- | :--- |
| Pin 1 (RS485 A+) | White/Orange | A / + | - |
| Pin 2 (RS485 B-) | Orange | B / - | - |
| Pin 3 (RS485 A+) | White/Green | A / + | - |
| Pin 4 (RS485 B-) | Blue | B / - | - |
| Pin 5-8 | *(cut and insulate)* | **DO NOT CONNECT** | **DO NOT CONNECT** |
| - | - | DI (TX) | GPIO_TX |
| - | - | RO (RX) | GPIO_RX |
| - | - | DE/RE | GPIO_FLOW |
| - | - | VCC | 3.3V / 5V |
| - | - | GND | GND |

The easiest approach is to take a standard ethernet cable, cut off one end, and identify the wires by color (T568B standard). Connect only the four data wires and leave the other four completely disconnected.

## Flow Control

The DE/RE pins on most RS485 modules need to be tied together and connected to a GPIO for flow control. The component automatically switches this pin HIGH during transmit and LOW during receive.

RS-485 is half-duplex, meaning only one device can talk at a time. The flow control pin tells the MAX485 transceiver when to switch between transmit and receive mode. The component handles this automatically with proper timing (500 microsecond turnaround delay after transmit).

## Communication Settings

| Parameter | Value |
| :--- | :--- |
| **Baud Rate** | 19200 |
| **Data Bits** | 8 |
| **Parity** | Even |
| **Stop Bits** | 1 |
| **Default Address** | 1 |

## ESP8266 Notes

The ESP8266 (including D1 Mini / NodeMCU) is supported but requires special configuration:

1. **Logger baud rate**: The ESP8266 has only one fully functional UART. You **must** set `logger: baud_rate: 0` to disable serial logging so the UART is available for RS-485 communication:

   ```yaml
   logger:
     baud_rate: 0
   ```

2. **Pin selection**: The default GPIO pins (16/17) are ESP32 pins. For ESP8266, use appropriate pins (e.g., GPIO1/GPIO3 for hardware UART, or software serial pins).

3. **Memory**: The ESP8266 has limited RAM. If you experience stability issues, consider disabling sensors you don't need by commenting them out in your YAML.

## Community Hardware Projects

- [**Project Box for WaterFurnace Aurora**](https://github.com/benpeart/esphome_waterfurnace_aurora_projectbox) by [Ben Peart](https://github.com/benpeart) — Parts list, assembly instructions, and a 3D-printable project box to house the ESP8266 + MAX485 module.
