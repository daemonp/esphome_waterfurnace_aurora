# WaterFurnace Aurora for ESPHome

A native ESPHome custom component for controlling and monitoring WaterFurnace geothermal heat pumps. This component communicates directly with the Aurora Base Control (ABC) board via RS-485 Modbus RTU, bypassing the need for expensive Symphony/AWL hardware or complex Raspberry Pi bridges.

It is a C++ port of the [waterfurnace_aurora Ruby gem](https://github.com/ccutrer/waterfurnace_aurora), optimized for ESP32/ESP8266 devices.

## Features

- **Climate Control**: Full thermostat integration (Heat, Cool, Auto, Emergency Heat, Fan modes).
- **Comprehensive Monitoring**: Reads entering/leaving water temps, air coil temps, line voltage, and airflow.
- **Energy Usage**: Reports power consumption (Watts) for compressor, blower, aux heat, and loop pump (requires performance monitoring kit).
- **Diagnostics**: Decodes fault codes into text (e.g., "High Pressure", "Condensate Overflow") and reports lockout status.
- **Variable Speed Data**: Supports 5-Series and 7-Series variable speed drive metrics (RPM, pressures, superheat/subcooling).
- **Efficient Communication**: Uses native WaterFurnace Modbus extensions (Func 0x41/0x42) for fast, low-latency updates.

## Hardware Requirements

1. **ESP32 or ESP8266 Development Board** (e.g., Wemos D1 Mini, NodeMCU).
2. **TTL to RS485 Adapter** (MAX485 or similar 3.3V compatible module).
3. **RJ45 Connector/Cable**: To connect to the AID Tool port on the heat pump.

### Wiring

Connect the RS485 adapter to the ESP and the Heat Pump's AID Tool port (RJ45).

| Heat Pump (RJ45) | Wire Color (T568B) | RS485 Adapter | ESP32/8266 |
| :--- | :--- | :--- | :--- |
| Pin 1 (RS485+) | White/Orange | A / + | - |
| Pin 2 (RS485-) | Orange | B / - | - |
| Pin 3 (RS485+) | White/Green | A / + | - |
| Pin 4 (RS485-) | Blue | B / - | - |
| - | - | DI (TX) | GPIO_TX |
| - | - | RO (RX) | GPIO_RX |
| - | - | DE/RE | GPIO_FLOW (Optional*) |
| - | - | VCC | 3.3V / 5V |
| - | - | GND | GND |

*Note: Many cheap RS485 modules require DE/RE pins to be pulled high/low to switch between Transmit and Receive. You can tie them to a single GPIO defined as `flow_control_pin` in the configuration.*

## Installation & Configuration

The recommended way to use this component is via **ESPHome Packages**. This allows you to include the full configuration with a single line and customize it using substitutions.

### Minimal Example

Add the following to your ESPHome YAML configuration:

```yaml
substitutions:
  name: waterfurnace-aurora
  friendly_name: "Geothermal Heat Pump"
  # Adjust pins to match your wiring
  uart_tx_pin: GPIO17
  uart_rx_pin: GPIO16
  flow_control_pin: GPIO4

packages:
  waterfurnace.aurora: github://daemonp/esphome_waterfurnace_aurora/waterfurnace_aurora.yaml@master

esphome:
  name: ${name}
  friendly_name: ${friendly_name}

esp32:
  board: esp32dev
  framework:
    type: arduino

# Standard ESPHome components
logger:
api:
ota:
wifi:
  ssid: "MyWiFi"
  password: "password"
```

### Configuration Variables

You can customize the following variables in the `substitutions` section:

| Variable | Default | Description |
| :--- | :--- | :--- |
| `name` | `waterfurnace-aurora` | Device name used for ESPHome ID. |
| `friendly_name` | `WaterFurnace Aurora` | Friendly name for Home Assistant. |
| `uart_tx_pin` | `GPIO17` | TX pin connected to RS485 module DI. |
| `uart_rx_pin` | `GPIO16` | RX pin connected to RS485 module RO. |
| `flow_control_pin` | `GPIO4` | Pin for RS485 DE/RE flow control. |
| `modbus_address` | `1` | Modbus slave address of the Aurora ABC board. |
| `update_interval` | `5s` | How often to poll the heat pump for data. |

## Protocol Details

This component implements a custom Modbus client. Standard Modbus libraries cannot read the Aurora registers efficiently because WaterFurnace uses:
1.  **Function Code 0x41 (Read Ranges):** Reads multiple disparate register blocks in one request.
2.  **Function Code 0x42 (Read Specific):** Reads a list of arbitrary, non-sequential register addresses.

This allows the component to fetch ~50 data points in just a few transactions, keeping the bus responsive.

## Credits

Huge thanks to [Cody Cutrer (ccutrer)](https://github.com/ccutrer) for reverse engineering the protocol and creating the original Ruby implementation.
