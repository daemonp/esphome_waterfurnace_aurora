# Hardware Setup & Wiring

## Requirements

1. **ESP32 or ESP8266 Development Board** (e.g., Wemos D1 Mini, NodeMCU, ESP32-DevKit)
2. **TTL to RS485 Adapter** with manual DE/RE flow control pins (see below)
3. **RJ45 Connector/Cable** to connect to the AID Tool port on the heat pump

## Recommended RS485 Module

Use a MAX485 module with **exposed DE/RE pins** for manual flow control. A known working module is the [Alinan MAX485 RS485 Transceiver Module](https://www.amazon.com/dp/B00NIOLNAG).

> **Important**: Avoid RS485 modules with "automatic flow control" that only have VCC/TXD/RXD/GND pins. These lack the DE/RE pins needed for reliable Modbus RTU timing and will cause communication errors.

## AID Tool Port Pinout

The AID Tool port on the front of your heat pump uses an RJ45 jack. The 8 pins carry **two completely different buses** — RS-485 data and 24VAC thermostat power. You **must** understand which is which before connecting anything.

### RJ45 Plug Pin Numbering

When building your cable, you need to know how pin numbers map to wire positions. Hold the RJ45 plug with the **clip on top** and the gold contacts facing you. **Pin 1 is on the left**:

```
                 RJ45 Plug
  (clip on top, contacts facing you)

                 +-----------+
                 |   clip    |
  +--------------+-----------+---------------+
  |                                          |
  |  +----+----+----+----+----+----+----+----+
  |  | 1  | 2  | 3  | 4  | 5  | 6  | 7  | 8  |
  |  |W/Or| Or |W/Gn| Bl |W/Bl| Gn |W/Br| Br |
  |  +----+----+----+----+----+----+----+----+
  |                                          |
  +------------------------------------------+
                       |
                    to cable

  T568B wire order (left to right):
  1: White/Orange  2: Orange      3: White/Green  4: Blue
  5: White/Blue    6: Green       7: White/Brown  8: Brown
```

### Heat Pump AID Tool Port

Looking at the RJ45 **jack** on the heat pump (the port you plug into), the pin order is **mirrored** — pin 1 is on the right:

```
  Heat Pump AID Tool Port (looking at the jack)

  +------------------------------------------+
  |                                          |
  |  +----+----+----+----+----+----+----+----+
  |  | 8  | 7  | 6  | 5  | 4  | 3  | 2  | 1  |
  |  +----+----+----+----+----+----+----+----+
  |                                          |
  |             +=============+              |
  |             |  AID Tool   |              |
  +-------------+=============+--------------+
```

When you plug in, pin 1 on the plug mates with pin 1 on the jack — the mirroring is handled by the connector geometry. Just make sure you have the right wires on the right pins of the plug.

### Pin Assignment

```
  Pin   Signal   Wire Color (T568B)   Function
  ---   ------   ------------------   -------------------------
   1      A+     White/Orange         RS-485 data  <- CONNECT
   2      B-     Orange               RS-485 data  <- CONNECT
   3      A+     White/Green          RS-485 data  <- CONNECT
   4      B-     Blue                 RS-485 data  <- CONNECT
   5      R      White/Blue           24VAC power  !! DO NOT USE
   6      C      Green                24VAC power  !! DO NOT USE
   7      R      White/Brown          24VAC power  !! DO NOT USE
   8      C      Brown                24VAC power  !! DO NOT USE
```

Pins 1 & 3 are both A+ (tied together), and pins 2 & 4 are both B- (tied together). This is redundant by design — you only need one pair, but connecting both is fine.

> ### ⚠️ DANGER: 24VAC Power on Pins 5, 6, 7, 8
>
> Pins 5-8 carry **24VAC thermostat bus power** (C and R lines). These are **NOT** data pins.
>
> **DO NOT connect these pins to anything** — not to your RS-485 adapter, not to your ESP, not to ground, not to each other. If you short a 24VAC pin against a data pin, a ground wire, or anything else:
>
> - **Best case**: You blow the 3A automotive fuse inside the heat pump (replaceable, but you'll need to find it)
> - **Worst case**: You destroy the ABC (Aurora Base Control) board — a very expensive repair on a $40K+ heat pump
>
> If you're cutting an ethernet cable, strip and connect only the white/orange, orange, white/green, and blue wires. **Cut the remaining four wires short and tape or heat-shrink them** so they can't accidentally touch anything.

## Building the Cable

The easiest approach: take a standard ethernet cable (T568B), cut off one end, and connect only the four data wires. Leave the RJ45 plug on the other end to plug into the heat pump.

```
  RJ45 Plug                              MAX485          ESP
  (to heat pump)     Ethernet Cable      Module          Board
  +-----------+                          +---------+     +-----------+
  |Pin 1  A+  |-- White/Orange --+       |         |     |           |
  |Pin 2  B-  |-- Orange --------+       |         |     |           |
  |Pin 3  A+  |-- White/Green ---+------>| A+      |     |           |
  |Pin 4  B-  |-- Blue ----------+------>| B-      |     |           |
  |           |                          |         |     |           |
  |Pin 5   R  |.. White/Blue  \          |         |     |           |
  |Pin 6   C  |.. Green        \         |         |     |           |
  |Pin 7   R  |.. White/Brown  / CUT &   |         |     |           |
  |Pin 8   C  |.. Brown       /  TAPE!   |         |     |           |
  +-----------+                          |         |     |           |
               DO NOT USE pins 5-8!      | DI (TX) |---->| GPIO_TX   |
               24VAC - DANGER!           | RO (RX) |---->| GPIO_RX   |
                                         | DE + RE |---->| GPIO_FLOW |
                                         | VCC     |---->| 3.3V      |
                                         | GND     |---->| GND       |
                                         +---------+     +-----------+
```

**Cable assembly steps:**

1. Take a standard ethernet patch cable (T568B crimping)
2. Cut off one end, leaving the RJ45 plug on the heat pump end
3. Strip back the outer jacket ~2 inches
4. Identify the wires by color:
   - **Connect**: White/Orange + White/Green (both A+) → twist together → RS485 module **A / +**
   - **Connect**: Orange + Blue (both B-) → twist together → RS485 module **B / -**
   - **Cut short & insulate**: White/Blue, Green, White/Brown, Brown (24VAC — **DANGER**)
5. Plug the RJ45 end into the AID Tool port on the heat pump

## Wiring Summary

| Heat Pump (RJ45) | Wire Color (T568B) | RS485 Adapter | ESP32/8266 |
| :--- | :--- | :--- | :--- |
| Pin 1, 3 (RS485 A+) | White/Orange, White/Green | A / + | - |
| Pin 2, 4 (RS485 B-) | Orange, Blue | B / - | - |
| Pin 5-8 (24VAC) | *(cut and insulate!)* | **DO NOT CONNECT** | **DO NOT CONNECT** |
| - | - | DI (TX) | GPIO_TX |
| - | - | RO (RX) | GPIO_RX |
| - | - | DE + RE (tied) | GPIO_FLOW |
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
