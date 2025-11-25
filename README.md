# WaterFurnace Aurora for ESPHome

A native ESPHome custom component for controlling and monitoring WaterFurnace geothermal heat pumps. This component communicates directly with the Aurora Base Control (ABC) board via RS485, bypassing the need for expensive Symphony/AWL hardware or complex Raspberry Pi bridges.

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
| - | - | DE/RE | GPIO_EN (Optional*) |
| - | - | VCC | 3.3V / 5V |
| - | - | GND | GND |

*Note: Many cheap RS485 modules require DE/RE pins to be pulled high/low to switch between Transmit and Receive. You can tie them to a single GPIO defined as `flow_control_pin` in ESPHome.*

## Installation

1. Copy the `components` directory into your ESPHome project folder.
2. Add the configuration to your ESPHome YAML file.

## Configuration Example

```yaml
esphome:
  name: waterfurnace
  platform: ESP32

# Enable logging
logger:
  level: DEBUG
  baud_rate: 0  # Disable UART logging if using the same UART for RS485

# Define UART for RS485
uart:
  id: modbus_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 19200
  stop_bits: 1

# Load the component
waterfurnace_aurora:
  uart_id: modbus_uart
  # flow_control_pin: GPIO4  # Uncomment if your RS485 adapter requires it

# Expose the Climate entity
climate:
  - platform: waterfurnace_aurora
    name: "Geothermal Heat Pump"

# Expose Sensors
sensor:
  - platform: waterfurnace_aurora
    entering_air_temperature:
      name: "Entering Air Temp"
    leaving_air_temperature:
      name: "Leaving Air Temp"
    entering_water_temperature:
      name: "Entering Water Temp"
    leaving_water_temperature:
      name: "Leaving Water Temp"
    total_watts:
      name: "Total Power"
    compressor_watts:
      name: "Compressor Power"
    blower_watts:
      name: "Blower Power"
    aux_watts:
      name: "Aux Heat Power"
    loop_pressure:
      name: "Loop Pressure"
    compressor_speed:
      name: "Compressor Speed"
    fault_code:
      name: "Fault Code"

# Expose Text Sensors
text_sensor:
  - platform: waterfurnace_aurora
    current_mode:
      name: "Current Mode"
    fault_description:
      name: "Fault Description"

# Expose Binary Sensors
binary_sensor:
  - platform: waterfurnace_aurora
    compressor_running:
      name: "Compressor Running"
    aux_heat_running:
      name: "Aux Heat Running"
    blower_running:
      name: "Blower Running"
    lockout:
      name: "System Lockout"
```

## Protocol Details

This component implements a custom Modbus client. Standard Modbus libraries cannot read the Aurora registers efficiently because WaterFurnace uses:
1.  **Function Code 0x41 (Read Ranges):** Reads multiple disparate register blocks in one request.
2.  **Function Code 0x42 (Read Specific):** Reads a list of arbitrary, non-sequential register addresses.

This allows the component to fetch ~50 data points in just a few transactions, keeping the bus responsive.

## Credits

Huge thanks to [Cody Cutrer (ccutrer)](https://github.com/ccutrer) for reverse engineering the protocol and creating the original Ruby implementation.
