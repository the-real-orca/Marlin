# Tronxy X5SA

## Features

- CoreXY
- dual Z axis
- dual extruder + servo
- 2 filament sensors
- 2(1) hotend fans
- part cooling fan
- BL touch
- heat bed
- chamber heater + temp sensor
- chamber fan
- light

## Power

- Power Supply: 24V / 15A / 360W
- System:           20W
- Motors:           16W
- Hotend (default): 60W
- Hotend (dual):    100W
- Heat Bed:         320W
- Heat Chamber:     200W
- Raspberry Pi:     5V

## Mainboard

- extruders: 5 (+1)
- heaters: 2 hotend + 1 bed + 1 chamber -> 4
- temp sensors: 2 hotend + 1 bed + 1 chamber -> 4
- fans: 1 hotend / board + 1 part + 1 chamber -> 3
- filament sensors: 2
- BLtouch
- light

### SKR PRO

- CPU: 32-bit ARM Cortex-M4 STM32F407ZGT6
- Motor Driver: 6 (dual Z output)
- Bed Heater: 1 (dedicated power)
- HotEnd Heater: 3
- Temp. Inputs: 4
- Fan Outputs: 3
- Additional Power Outputs: -
- Endstops: 3
- Filament Sensor Inputs: 3
- BLTouch / Bed Probe: 1
- Additional I/Os: 18 ADCs: 6
- I2C: 1
- UART: 1
- SPI: 1

### SKR GTR

- CPU: 32-bit ARM Cortex-M4 STM32F407IGT6
- Motor Driver: 6 (dual Z output)
- Bed Heater: 1 (dedicated power)
- HotEnd Heater: 3
- Temp. Inputs: 4 + 1 K-Temp
- Fan Outputs: 3
- Additional Power Outputs: -
- Endstops: 5
- Filament Sensor Inputs: 3
- BLTouch / Bed Probe: 1
- Additional I/Os: 18 ADCs: 6
- NeoPixel: 1
- CAN Bus: 2
- I2C: 1

## Toolhead System

### Required Signals

- Hotend 0:     2 (4A)
- Temp Sensors: 2
- Hotend Fan:   2
- Part Fan:     2
- BL Touch:     4
- **Sum:       12**

- Hotend 1:     2 (4A)
- Temp Sensors: 2
- Hotend Fan:   2
- Tool Switch:  3
- **Sum:        9**

- Extruder:     4 (1.5A)
- Filament Sensor: 2
- **Sum:        6**


### Electrical Connection: DVI

|||||||||||
|----|----|----|----|----|----|----|----|-|--|
|  1 |  2 |  3 | (4)| (5)|  6 |  7 |  8 |
|  9 | 10 | 11 |(12)|(13)| 14 | 15 | 16 | |C5|
| 17 | 18 | 19 |(20)|(21)| 22 | 23 | 24 |

#### Twisted Pair Data:

- 1&2, 3 Shield, (4&5)
- 9&10, 11 Shield, (12&13)
- 17&18, 19 Shield, (20&21)
- Clock: 23&24, 22 Shield

#### Signals:

- 14: +5V
- 15: GND
- 16: HotPlug
- 6: DDC Clock
- 7: DDC Data
- 8: V-Sync
- C5: GND

#### Toolhead Pins

- C5: GND
- 1&2: Hotend 0
- 3: (Shield)
- 4&5: Hotend 1
- 6, 7: Part Fan
- 8: BL Touch
- 9,10: Hotend 0
- 11: (Shield)
- 12,13: Hotend 1
- 14: +5V
- 15: GND
- 16: Servo
- 17,18: Hotend Fan
- 19: (Shield)
- 20, 21: Tool Switch
- 22: GND Temp Sensor
- 23: Temp Sensors 0
- 24: Temp Sensors 1

### VGA

||||||
|----|----|----|----|----|
|  1 |  2 |  3 |  4 |  5 |
|  6 |  7 |  8 |  9 | 10 |
| 11 | 12 | 13 | 14 | 15 |

- 1: Rot (0,7 Vss @ 75 Ohm)
- 2: Grün (Pegel & Impedanz s. o.) oder (analoges) Monochromsignal
- 3: Blau (Pegel & Impedanz s. o.)
- 4: ID2 (Monitor-IDentifizierung #2) oder Reserve
- 5: digitale Masse für DDC
- 6: analoge Masse für Rot
- 7: analoge Masse für Grün
- 8: analoge Masse für Blau
- 9: nicht belegt; glgt. +5 V von der Grafikkarte (= Computer angeschlossen)
- 10: analoge Masse für Synchronsignale
- 11: ID0
- 12: ID1 oder digit. Datenaustausch (DDC – SDA, Serial DAta = Datenleitung)
- 13: Horizontale Synchronisation oder gemeinsame Sync. (H + V)
- 14: Vertikale Synchronisation
- 15: Digit. Datenaustausch (DDC – SCL, Serial CLock = Taktleitung)

#### Toolhead Pins

- 1&2, 6&7: Hotend 0
- 3,8: Hotend Fan
- 4: Servo
- 5: GND
- 9: +5V
- 10: GND
- 11, 12: Part Fan
- 13, 14: Temp Sensor
- 15: BL Touch


## Housing

### Heating

**100W Heater**
Umbau auf 24V, Fan mit 20R in Serie
==> 10cm Abstand, offen ==> 38°C

**150W Heater**
24V, Fan mit 20R (22V) in Serie
==> 82°C
10cm Abstand, offen ==> 51°C

Fan mit 100R (16V) in Serie
==> 10cm Abstand, offen ==> 61°C

