# NodeMCU Home Control Station
A NodeMCU (ESP8266) project that demonstrates a small home automation stack:
- Web UI with role-based (Admin / Guest) access
- Gesture control (HC-SR04 ultrasonic) — wave to toggle the fan
- Controls: LED (status), Fan, Window motor open/close, Appliance 1 & 2 (relays)
- Arduino IDE compatible `.ino` file (no command line required)

## Files
- `home_control_station.ino` — main Arduino IDE sketch

## Features
- **Guest access**: can toggle the LED and view status.
- **Admin access**: can toggle fan, appliances, operate window (open/close/stop), and change passwords.
- **Gesture**: a quick hand wave within ~25 cm toggles the fan (adjustable in code).
- Safety: window controls have runtime limits to prevent motor damage — add limit switches for production.

## Hardware (example)
- NodeMCU ESP8266 (ESP-12E)
- HC-SR04 ultrasonic sensor (Trig -> D6, Echo -> D7)
- Relay module(s) (for Fan, Window motor, Appliances) — connect control pins to D1..D5
- LED (status) -> D0
- Common ground between NodeMCU and sensors/relays
- If using a motor for the window: use motor driver or H-bridge; DO NOT drive motor directly from NodeMCU pins.
- Consider flyback diodes and proper power supply for relays & motors.

### Suggested pin mapping (as used in code)
- D0 — Status LED
- D1 — Fan relay
- D2 — Window open relay
- D3 — Window close relay
- D4 — Appliance 1 relay
- D5 — Appliance 2 relay
- D6 — HC-SR04 Trig
- D7 — HC-SR04 Echo

## Software / Libraries
No additional 3rd-party libraries required beyond:
- `ESP8266WiFi`
- `ESP8266WebServer`
- (Both available with Arduino ESP8266 board package)

## Usage
1. Open `home_control_station.ino` in Arduino IDE.
2. Set your WiFi SSID & password at the top of the file.
3. Adjust any pins or passwords if needed.
4. Select `NodeMCU 1.0 (ESP-12E Module)` as board and upload.
5. Access the device:
   - If connected to your WiFi, open `http://<device-ip>/` on any browser (Windows or other).
   - If WiFi connection fails, the board runs as an Access Point `HomeControlAP`. Connect to it and open `http://192.168.4.1/`.
6. Login with password:
   - Admin default: `admin123`
   - Guest default: `guest123`
   (Change them in the Admin dashboard; persistent storage not implemented in this version.)

## Safety
- For the window motor: add hardware limit switches and use them in code to stop motion precisely.
- Use separate power supplies for high-current devices (fans, motors). Keep grounds common.
- Add snubber diodes for inductive loads, opto-isolated relays if possible.

## Next steps / Improvements
- Save passwords and device states to EEPROM or SPIFFS for persistence across reboots.
- Add SSL / HTTPS for safer password handling.
- Add an authentication token (instead of plain-text GET password).
- Integrate with MQTT / Home Assistant.
- Use APDS9960 or an IMU for improved gesture recognition.
- Add a small OLED / LCD local UI for local control.

## License
MIT — adapt, reuse, and contribute back!
