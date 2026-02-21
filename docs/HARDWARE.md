# Hardware Setup & Wiring

## Requirements

1. **ESP32 or ESP8266 Development Board** (e.g., Wemos D1 Mini, NodeMCU, ESP32-DevKit)
2. **TTL to RS485 Adapter** with manual DE/RE flow control pins (see below)
3. **RJ45 Connector/Cable** to connect to the AID Tool port on the heat pump

## Recommended RS485 Module

Use a MAX485 module with **exposed DE/RE pins** for manual flow control. A known working module is the [Alinan MAX485 RS485 Transceiver Module](https://www.amazon.com/dp/B00NIOLNAG).

> **Important**: Avoid RS485 modules with "automatic flow control" that only have VCC/TXD/RXD/GND pins. These lack the DE/RE pins needed for reliable Modbus RTU timing and will cause communication errors.

## Wiring

Connect the RS485 adapter to the ESP and the Heat Pump's AID Tool port (RJ45).

| Heat Pump (RJ45) | Wire Color (T568B) | RS485 Adapter | ESP32/8266 |
| :--- | :--- | :--- | :--- |
| Pin 1 (RS485+) | White/Orange | A / + | - |
| Pin 2 (RS485-) | Orange | B / - | - |
| Pin 3 (RS485+) | White/Green | A / + | - |
| Pin 4 (RS485-) | Blue | B / - | - |
| - | - | DI (TX) | GPIO_TX |
| - | - | RO (RX) | GPIO_RX |
| - | - | DE/RE | GPIO_FLOW |
| - | - | VCC | 3.3V / 5V |
| - | - | GND | GND |

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
