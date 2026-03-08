# Wiring Guides

## Node 1 — ESP32 Feather V2 + MPU-6050 + ArduCAM Mini SPI

### I²C — MPU-6050

| MPU-6050 Pin | ESP32 Feather V2 Pin | Notes |
|---|---|---|
| VCC | 3V3 | 3.3 V supply |
| GND | GND | Common ground |
| SDA | GPIO 22 | 4.7 kΩ pull-up to 3V3 |
| SCL | GPIO 20 | 4.7 kΩ pull-up to 3V3 |
| AD0 | GND | I²C address = 0x68 |
| INT | GPIO 35 (optional) | Interrupt-driven sampling |

### SPI — ArduCAM Mini OV2640

| ArduCAM Pin | ESP32 Feather V2 Pin |
|---|---|
| VCC | 3V3 |
| GND | GND |
| MOSI | GPIO 18 |
| MISO | GPIO 19 |
| SCK | GPIO 5 |
| CS | GPIO 33 |
| SDA (I²C) | GPIO 22 (shared with MPU-6050) |
| SCL (I²C) | GPIO 20 (shared with MPU-6050) |

> **Note:** ArduCAM uses I²C for register config and SPI for image data.
> Both peripherals share the I²C bus — ensure MPU-6050 address (0x68) and
> ArduCAM OV2640 address (0x30) do not conflict.

## Nodes 2–5 — ESP32 Feather V2 + MPU-6050 only

Same I²C wiring as Node 1 minus the ArduCAM.
