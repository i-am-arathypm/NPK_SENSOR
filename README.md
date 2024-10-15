# NPK Sensor Implementation Using ULP LoRa, RS-485, and JXCT NPK Sensor

This project is an NPK (Nitrogen, Phosphorus, Potassium) sensor implementation using a low-power setup with ULP LoRa, RS-485 communication, and the JXCT NPK sensor. The system gathers soil nutrient data and sends it wirelessly via LoRaWAN. This setup uses an Arduino Pro Mini with the RFM95W LoRa module, the JXCT NPK sensor connected through an RS-485 module, and is powered by batteries to support low-power operation.

## Components Used
- **ULP LoRa** (Arduino Pro Mini with RFM95W LoRa Module)
- **JXCT NPK Sensor** (for measuring Nitrogen, Phosphorus, and Potassium)
- **RS-485 Communication Module** (for sensor communication)
- **MOSFET and Buck Converter** (for power management and sleep mode)
- **4.2V Batteries** (for powering different modules)

## Features
- Low-power implementation using the `Low-Power` library to enable sleep modes in the Arduino Pro Mini.
- LoRaWAN communication via the RFM95W module using the `MCCI_LoRaWAN_LMIC` library.
- Modbus communication protocol over RS-485 to read data from the JXCT NPK sensor using the `ModbusMaster` library.
- Power optimization with MOSFET to turn off components during sleep mode.

## Libraries Used
The required libraries are included in the **`libraries`** folder of this repository:
- **Low-Power-master**: For enabling sleep mode on the Arduino Pro Mini.
- **MCCI_LoRaWAN_LMIC_library**: For LoRaWAN communication.
- **ModbusMaster-2.0.1**: For RS-485 Modbus communication with the NPK sensor.


## Hardware Connections
- **ULP LoRa**:
  - Powered by one 4.2V battery.
  - RS-485 communication module connected to the JXCT NPK sensor.

- **Buck Converter**:
  - Connected to two 4.2V batteries in series (providing 8V input).
  - Outputs 5V to power the RS-485 module and the JXCT NPK sensor.

- **MOSFET**:
  - Used to control power to the RS-485 module during sleep mode.

## Code Overview
1. **LoRa Communication**: The `MCCI_LoRaWAN_LMIC` library is used for LoRaWAN communication. It handles packet transmission and network joining.
2. **Low-Power Mode**: The `Low-Power` library puts the Arduino into a deep sleep state to conserve battery when not transmitting or reading data.
3. **Modbus Protocol**: The `ModbusMaster` library facilitates RS-485 communication with the JXCT NPK sensor, allowing the Arduino to read sensor values.

## How It Works
1. The system wakes up periodically and reads NPK values from the JXCT sensor over RS-485.
2. The sensor values are transmitted wirelessly via LoRaWAN.
3. The system enters a low-power sleep mode when not in use to extend battery life.

## Power Management
- The Arduino Pro Mini operates in an ultra-low-power mode, while the MOSFET ensures the RS-485 module is only powered when the sensor data is being collected.
- The use of a buck converter ensures stable 5V supply to the sensor and communication module, even though the main power source is 8V (from two 4.2V batteries in series).
