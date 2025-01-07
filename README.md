# Raspberry Pi Pico and ENC28J60 Connections

| **Signal Name** | **Raspberry Pi Pico**     | **Pin Nº**      | **Direction** | **ENC28J60 Pin Name** |
|-----------------|---------------------------|-----------------|---------------|-----------------------|
| **MISO**        | GPIO 16                   | Pin 21          | **<---**      | SO                    |
| **MOSI**        | GPIO 19                   | Pin 25          | **--->**      | SI                    |
| **CLK**         | GPIO 18                   | Pin 24          | **--->**      | SCK                   |
| **ENC_RST**     | GPIO 21                   | PIN 27          | **--->**      | RESET                 |
| **ENC_CS**      | GPIO 17                   | PIN 22          | **--->**      | CS                    |
| **IRQ**         | GPIO 22                   | PIN 29          | **<---**      | INT                   |
| **3V3(OUT)**    |                           | PIN 36          | **--->**      | VCC                   |
| **GND**         |                           | PIN 38          | **--->**      | GND                   |

