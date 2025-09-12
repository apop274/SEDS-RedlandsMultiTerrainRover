# SEDS-RedlandsMultiTerrainRover


# Simpler Rover

ESP32 + E32 LoRa link and DFR0601 motor drivers.

## Hardware
- ESP32-DevKitC
- Ebyte E32-915T20D (UART LoRa, 3.3V)
- DFRobot DFR0601 (dual 12A motor driver) x2 (rover only)

## Wiring (radio, both sides)
- E32 VCC -> ESP32 3V3
- E32 GND -> ESP32 GND
- E32 TXD -> ESP32 GPIO16 (RX2)
- E32 RXD -> ESP32 GPIO17 (TX2)
- E32 M0 -> GND, M1 -> GND
- AUX optional (GPIO34)

## Build
- VS Code + PlatformIO
- Control env: bridges USB<->LoRa (heartbeat)
- Rover env: prints/echoes received bytes (or drives motors in rover build)

## Notes
- Character-mode terminal @ 115200 for control.
- E32 UART @ 9600.
- Antennas on before power.
