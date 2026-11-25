# AA CAN node v2 pinout

These assignments follow the STM32F103RET6 (U15) and IIS2MDCTR (U1) on the EasyEDA PCB named `PCB_Wing PCB Module Compass copy copy`, in the project `Can Node M9- 2026 v3 w IIS2MDC`. The PX4 target is `aa_cannode_v2_default`.

## External UART5 and I²C connector

Pin numbers follow the numbered PCB pads on the six-pin connector marked `UART5`. Viewed by signal destination, pins 2 through 5 connect to the peripheral's **RX, TX, SCL, SDA**, in that order.

| Connector pin | Board net | STM32F103RE pin | Connect to peripheral |
|---|---|---|---|
| 1 | 5V-CAN | N/A | 5 V rail |
| 2 | UART5_TX | PC12, U15 pad 53 | RX |
| 3 | UART5_RX | PD2, U15 pad 54 | TX |
| 4 | SCL | PB8, U15 pad 61 | SCL |
| 5 | SDA | PB9, U15 pad 62 | SDA |
| 6 | GND | N/A | GND |

The serial and I²C signals use 3.3 V logic. UART5 is available as `/dev/ttyS2` at 115200 baud by default. UART4 (`/dev/ttyS1`, PC10 TX and PC11 RX) serves the on-board NEO-M9N. USART2 (`/dev/ttyS0`, PA2 TX and PA3 RX) serves the debug connector U12.

## On-board compass

U1 uses remapped I²C1 on the same SCL and SDA nets as connector pins 4 and 5. The 7-bit IIS2MDC address is `0x1E`.

| U1 pad | Signal | Connection |
|---|---|---|
| 1 | SCL | PB8 |
| 3 | CS | 3.3 V, selecting I²C |
| 4 | SDA | PB9 |
| 7 | INT/DRDY | Open. The driver polls for samples. |

## Compass mounting

Mount the node component side down with the printed arrow pointing toward the aircraft nose. The IIS2MDC driver applies Roll 180 and Yaw 270 (PX4 rotation 14). Set the external compass rotation on the flight controller to No rotation for this installation. Set the flight controller board rotation to match its own mounting.
