/*
  home_control_station.ino
  NodeMCU (ESP8266) Weather/Appliance + Gesture + Web Access Example
  - Web UI with Admin / Guest roles
  - Gesture control using HC-SR04 (hand wave)
  - Control LED, Fan, Window motor (relays), Appliances (relays)
  - No command-line build required (Arduino IDE friendly)
  - Long, commented file (300+ lines)

  Hardware assumptions:
  - NodeMCU (ESP8266)
  - HC-SR04 ultrasonic: trig -> D6, echo -> D7
  - LED (status): D0
  - Fan relay: D1
  - Window open relay: D2
  - Window close relay: D3
  - Appliance 1 relay: D4
  - Appliance 2 relay: D5
  - Optional push button for manual toggle: A0 read via voltage divider or use digital pin
  - Use common GND for sensors and NodeMCU
  - Relay boards should be connected through proper transistor drivers or opto-isolated relay boards

  Author: Generated for user request
  Date: 2025
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>

// ----------------------- USER CONFIG ---------------------- //
// Change these as needed:
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";

// Web UI passwords (simple; extend with hashing & EEPROM for real projects)
String ADMIN_PASSWORD = "admin123";     // admin - full control
String GUEST_PASSWORD = "guest123";     // guest - limited control

// Web server port
const int WEB_PORT = 80;

// ----------------------- PIN DEFINITIONS ------------------ //
#define PIN_LED_STATUS D0    // Status LED
#define PIN_RELAY_FAN  D1    // Fan relay
#define PIN_RELAY_WIN_OPEN D2// Window open relay
#define PIN_RELAY_WIN_CLOSE D3// Window close relay
#define PIN_RELAY_AP1 D4     // Appliance 1
#define PIN_RELAY_AP2 D5     // Appliance 2

// HC-SR04 Ultrasonic for gesture
#define PIN_TRIG D6
#define PIN_ECHO D7

// Gesture parameters (tweak to adjust sensitivity)
const unsigned long GESTURE_SAMPLE_MS = 60;    // how frequently to sample (ms)
const int GESTURE_THRESHOLD_CM = 25;          // distance below which we consider a "wave" (cm)
const int GESTURE_MIN_DROP_CM = 15;           // minimum drop from baseline to count
const unsigned long GESTURE_COOLDOWN_MS = 1200; // after a gesture recognized, ignore for this ms

// Window motor safety (milliseconds)
const unsigned long WINDOW_MAX_RUN_MS = 10000; // don't run motor more than this (safety)
const unsigned long WINDOW_STOP_DELAY_MS = 500; // time to avoid quick switching

// ----------------------- GLOBALS -------------------------- //
ESP8266WebServer server(WEB_PORT);

// Gesture tracking
volatile unsigned long lastGestureTime = 0;
unsigned long lastSampleTime = 0;
int baselineDistance = -1;
bool gestureReady = true;

// Window motor state tracking
enum WinState { WIN_STOP, WIN_OPENING, WIN_CLOSING };
WinState windowState = WIN_STOP;
unsigned long windowActionStart = 0;

// Simple device states (for UI)
bool ledStatus = false;
bool fanStatus = false;
bool ap1Status = false;
bool ap2Status = false;

// helper ticker for status LED blink
Ticker statusTicker;
bool statusLedToggle = false;

// --------------- HELPER / HARDWARE FUNCTIONS -------------- //

void setPinOutputs() {
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_RELAY_FAN, OUTPUT);
  pinMode(PIN_RELAY_WIN_OPEN, OUTPUT);
  pinMode(PIN_RELAY_WIN_CLOSE, OUTPUT);
  pinMode(PIN_RELAY_AP1, OUTPUT);
  pinMode(PIN_RELAY_AP2, OUTPUT);

  // initialize relays off (assuming HIGH = ON; set accordingly)
  digitalWrite(PIN_LED_STATUS, LOW);
  digitalWrite(PIN_RELAY_FAN, LOW);
  digitalWrite(PIN_RELAY_WIN_OPEN, LOW);
  digitalWrite(PIN_RELAY_WIN_CLOSE, LOW);
  digitalWrite(PIN_RELAY_AP1, LOW);
  digitalWrite(PIN_RELAY_AP2, LOW);
}

void setRelay(int pin, bool on) {
  // If your relay is active LOW, invert "on" here
  digitalWrite(pin, on ? HIGH : LOW);
}

// Status LED blink function for Ticker
void tickStatusLED() {
  statusLedToggle = !statusLedToggle;
  digitalWrite(PIN_LED_STATUS, statusLedToggle ? HIGH : LOW);
}

// -------------------- ULTRASONIC FUNCTIONS ----------------- //
void ultrasonicTrigger() {
  // send 10us pulse
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
}

long ultrasonicReadCM() {
  ultrasonicTrigger();
  // measure echo pulse width (timeout to avoid blocking)
  long duration = pulseIn(PIN_ECHO, HIGH, 30000); // timeout 30 ms
  if (duration == 0) return -1;
  long distanceCm = duration / 58; // speed of sound calc
  return distanceCm;
}

// Gesture detection — called in loop frequently
void gestureTask() {
  unsigned long now = millis();
  if (now - lastSampleTime < GESTURE_SAMPLE_MS) return;
  lastSampleTime = now;

  long d = ultrasonicReadCM();
  if (d <= 0) {
    // sensor error or no echo; ignore but keep baseline if present
    return;
  }

  // establish baseline if not set
  if (baselineDistance < 0) {
    baselineDistance = d;
    return;
  }

  // If distance drops significantly from baseline -> possible gesture
  int drop = baselineDistance - d;
  if (drop >= GESTURE_MIN_DROP_CM && d <= GESTURE_THRESHOLD_CM) {
    // check cooldown
    if (millis() - lastGestureTime > GESTURE_COOLDOWN_MS) {
      lastGestureTime = millis();
      // perform a gesture action: toggle fan for example
      // (you can change mapping easily)
      fanStatus = !fanStatus;
      setRelay(PIN_RELAY_FAN, fanStatus);
      Serial.printf("Gesture detected! Toggled fan to %s\n", fanStatus ? "ON" : "OFF");
    }
  }

  // slowly adapt baseline (to account for environmental changes)
  // move baseline toward current reading by 1cm per sample if difference small
  if (abs(baselineDistance - d) < 6) {
    if (baselineDistance < d) baselineDistance++;
    else if (baselineDistance > d) baselineDistance--;
  } else {
    // if baseline drifted a lot, re-evaluate less aggressively
    baselineDistance = (baselineDistance * 9 + d) / 10;
  }
}

// ------------------- WINDOW CONTROL ---------------------- //
void windowOpen() {
  if (windowState == WIN_OPENING) return;
  Serial.println("Window opening requested");
  // Safety: stop any closing action
  digitalWrite(PIN_RELAY_WIN_CLOSE, LOW);
  delay(50);
  digitalWrite(PIN_RELAY_WIN_OPEN, HIGH);
  windowState = WIN_OPENING;
  windowActionStart = millis();
}

void windowClose() {
  if (windowState == WIN_CLOSING) return;
  Serial.println("Window closing requested");
  // Safety: stop any opening action
  digitalWrite(PIN_RELAY_WIN_OPEN, LOW);
  delay(50);
  digitalWrite(PIN_RELAY_WIN_CLOSE, HIGH);
  windowState = WIN_CLOSING;
  windowActionStart = millis();
}

void windowStop() {
  Serial.println("Window stop");
  digitalWrite(PIN_RELAY_WIN_OPEN, LOW);
  digitalWrite(PIN_RELAY_WIN_CLOSE, LOW);
  windowState = WIN_STOP;
  windowActionStart = 0;
}

// call periodically to enforce safety timeout
void windowSafetyTask() {
  if (windowState == WIN_OPENING || windowState == WIN_CLOSING) {
    if (millis() - windowActionStart > WINDOW_MAX_RUN_MS) {
      Serial.println("Window safety timeout — stopping motor");
      windowStop();
    }
  }
}

// ------------------- WEB UI & AUTH ----------------------- //

// Basic HTML page generation helpers
String pageHeader(const String &title) {
  String h = "<!doctype html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  h += "<title>" + title + "</title>";
  h += "<style>body{font-family:Arial,Helvetica,sans-serif;padding:8px;} .card{border-radius:8px;padding:12px;margin:8px 0;border:1px solid #ddd;} button{padding:8px 12px;margin:4px;} .on{color:green;font-weight:bold;} .off{color:red;font-weight:bold;} .small{font-size:12px;color:#555;}</style>";
  h += "</head><body>";
  h += "<h2>" + title + "</h2>";
  return h;
}

String pageFooter() {
  String f = "<hr><div class='small'>NodeMCU Home Control Station</div>";
  f += "</body></html>";
  return f;
}

bool checkAuth(String role) {
  // role = "admin" or "guest"
  // read url param ?pw=...
  if (!server.hasArg("pw")) return false;
  String pw = server.arg("pw");
  if (role == "admin") return (pw == ADMIN_PASSWORD);
  else return (pw == GUEST_PASSWORD || pw == ADMIN_PASSWORD);
}

// Handler: root -> show login / status
void handleRoot() {
  String html = pageHeader("Home Control - Login / Status");

  html += "<div class='card'><form action='/dashboard' method='GET'>";
  html += "<label>Enter Password (admin or guest):</label><br>";
  html += "<input name='pw' type='password' style='padding:6px;width:70%;'><br>";
  html += "<button type='submit'>Enter</button>";
  html += "</form></div>";

  html += pageFooter();
  server.send(200, "text/html", html);
}

// Handler: dashboard (requires password param)
void handleDashboard() {
  // If no password, redirect to root
  if (!server.hasArg("pw")) {
    server.sendHeader("Location", "/");
    server.send(302);
    return;
  }
  String pw = server.arg("pw");
  bool isAdmin = (pw == ADMIN_PASSWORD);
  bool isGuest = (pw == GUEST_PASSWORD) || isAdmin;

  if (!isGuest) {
    server.send(403, "text/plain", "Forbidden - invalid password");
    return;
  }

  String html = pageHeader("Control Dashboard");

  // show role
  html += "<div class='card'>Role: <strong>" + String(isAdmin ? "Admin" : "Guest") + "</strong></div>";

  // status display
  html += "<div class='card'><h3>Device Status</h3>";
  html += "<div>LED: " + String(ledStatus ? "<span class='on'>ON</span>" : "<span class='off'>OFF</span>") + "</div>";
  html += "<div>Fan: " + String(fanStatus ? "<span class='on'>ON</span>" : "<span class='off'>OFF</span>") + "</div>";
  html += "<div>Appliance 1: " + String(ap1Status ? "<span class='on'>ON</span>" : "<span class='off'>OFF</span>") + "</div>";
  html += "<div>Appliance 2: " + String(ap2Status ? "<span class='on'>ON</span>" : "<span class='off'>OFF</span>") + "</div>";
  html += "<div>Window State: " + String((windowState==WIN_OPENING) ? "OPENING" : (windowState==WIN_CLOSING) ? "CLOSING" : "STOPPED") + "</div>";
  html += "</div>";

  // Guest controls (toggle LED)
  html += "<div class='card'><h3>Guest Controls</h3>";
  html += "<form action='/toggleLED' method='GET'><input type='hidden' name='pw' value='" + pw + "'>";
  html += "<button type='submit'>" + String(ledStatus ? "Turn LED OFF" : "Turn LED ON") + "</button></form></div>";

  // Admin-only controls
  if (isAdmin) {
    html += "<div class='card'><h3>Admin Controls</h3>";
    // Fan
    html += "<form action='/toggleFan' method='GET' style='display:inline-block;margin-right:8px;'><input type='hidden' name='pw' value='" + pw + "'>";
    html += "<button type='submit'>" + String(fanStatus ? "Stop Fan" : "Start Fan") + "</button></form>";

    // Appliances
    html += "<form action='/toggleAP1' method='GET' style='display:inline-block;margin-right:8px;'><input type='hidden' name='pw' value='" + pw + "'>";
    html += "<button type='submit'>" + String(ap1Status ? "AP1 OFF" : "AP1 ON") + "</button></form>";

    html += "<form action='/toggleAP2' method='GET' style='display:inline-block;'><input type='hidden' name='pw' value='" + pw + "'>";
    html += "<button type='submit'>" + String(ap2Status ? "AP2 OFF" : "AP2 ON") + "</button></form>";

    // Window actions
    html += "<div style='margin-top:10px;'>";
    html += "<form action='/winOpen' method='GET' style='display:inline-block;margin-right:4px;'><input type='hidden' name='pw' value='" + pw + "'>";
    html += "<button type='submit'>Open Window</button></form>";

    html += "<form action='/winClose' method='GET' style='display:inline-block;margin-right:4px;'><input type='hidden' name='pw' value='" + pw + "'>";
    html += "<button type='submit'>Close Window</button></form>";

    html += "<form action='/winStop' method='GET' style='display:inline-block;'><input type='hidden' name='pw' value='" + pw + "'>";
    html += "<button type='submit'>Stop Window</button></form>";
    html += "</div>";
    html += "</div>";
  }

  // gesture info
  html += "<div class='card'><h3>Gesture</h3>";
  html += "<div class='small'>Wave your hand in front of the ultrasonic sensor to toggle the fan. Gesture detection threshold: " + String(GESTURE_THRESHOLD_CM) + "cm</div></div>";

  // password management (admin)
  if (isAdmin) {
    html += "<div class='card'><h3>Passwords</h3>";
    html += "<form action='/setPasswords' method='POST'>"
            "<label>Admin Password:</label><br>"
            "<input type='password' name='adminpw' placeholder='new admin pw'><br>"
            "<label>Guest Password:</label><br>"
            "<input type='password' name='guestpw' placeholder='new guest pw'><br>"
            "<input type='hidden' name='pw' value='" + pw + "'>"
            "<button type='submit'>Save</button>"
            "</form></div>";
  }

  html += pageFooter();
  server.send(200, "text/html", html);
}

// simple action endpoints
void handleToggleLED() {
  if (!server.hasArg("pw")) { server.send(403, "text/plain", "Forbidden"); return; }
  if (!checkAuth("guest")) { server.send(403, "text/plain", "Forbidden"); return; }
  ledStatus = !ledStatus;
  digitalWrite(PIN_LED_STATUS, ledStatus ? HIGH : LOW);
  server.sendHeader("Location", "/dashboard?pw=" + server.arg("pw"));
  server.send(302);
}

void handleToggleFan() {
  if (!server.hasArg("pw")) { server.send(403, "text/plain", "Forbidden"); return; }
  if (!checkAuth("admin")) { server.send(403, "text/plain", "Forbidden"); return; }
  fanStatus = !fanStatus;
  setRelay(PIN_RELAY_FAN, fanStatus);
  server.sendHeader("Location", "/dashboard?pw=" + server.arg("pw"));
  server.send(302);
}

void handleToggleAP1() {
  if (!server.hasArg("pw")) { server.send(403, "text/plain", "Forbidden"); return; }
  if (!checkAuth("admin")) { server.send(403, "text/plain", "Forbidden"); return; }
  ap1Status = !ap1Status;
  setRelay(PIN_RELAY_AP1, ap1Status);
  server.sendHeader("Location", "/dashboard?pw=" + server.arg("pw"));
  server.send(302);
}

void handleToggleAP2() {
  if (!server.hasArg("pw")) { server.send(403, "text/plain", "Forbidden"); return; }
  if (!checkAuth("admin")) { server.send(403, "text/plain", "Forbidden"); return; }
  ap2Status = !ap2Status;
  setRelay(PIN_RELAY_AP2, ap2Status);
  server.sendHeader("Location", "/dashboard?pw=" + server.arg("pw"));
  server.send(302);
}

void handleWinOpen() {
  if (!server.hasArg("pw")) { server.send(403, "text/plain", "Forbidden"); return; }
  if (!checkAuth("admin")) { server.send(403, "text/plain", "Forbidden"); return; }
  windowOpen();
  server.sendHeader("Location", "/dashboard?pw=" + server.arg("pw"));
  server.send(302);
}

void handleWinClose() {
  if (!server.hasArg("pw")) { server.send(403, "text/plain", "Forbidden"); return; }
  if (!checkAuth("admin")) { server.send(403, "text/plain", "Forbidden"); return; }
  windowClose();
  server.sendHeader("Location", "/dashboard?pw=" + server.arg("pw"));
  server.send(302);
}

void handleWinStop() {
  if (!server.hasArg("pw")) { server.send(403, "text/plain", "Forbidden"); return; }
  if (!checkAuth("admin")) { server.send(403, "text/plain", "Forbidden"); return; }
  windowStop();
  server.sendHeader("Location", "/dashboard?pw=" + server.arg("pw"));
  server.send(302);
}

// admin password change (POST)
void handleSetPasswords() {
  if (!server.hasArg("pw")) { server.send(403, "text/plain", "Forbidden"); return; }
  if (!checkAuth("admin")) { server.send(403, "text/plain", "Forbidden"); return; }

  if (server.hasArg("adminpw")) {
    String newA = server.arg("adminpw");
    if (newA.length() >= 4) {
      ADMIN_PASSWORD = newA;
      Serial.println("Admin password changed via web UI");
    }
  }
  if (server.hasArg("guestpw")) {
    String newG = server.arg("guestpw");
    if (newG.length() >= 3) {
      GUEST_PASSWORD = newG;
      Serial.println("Guest password changed via web UI");
    }
  }
  server.sendHeader("Location", "/dashboard?pw=" + server.arg("pw"));
  server.send(302);
}

// simple not found
void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

// ---------------------- SETUP ----------------------------- //
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("Starting Home Control Station...");

  setPinOutputs();

  // ultrasonic pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // start status LED ticker every 1s
  statusTicker.attach(1.0, tickStatusLED);

  // connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("Connecting to WiFi SSID: %s\n", ssid);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi, starting Access Point mode.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HomeControlAP");
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP: ");
    Serial.println(myIP);
  }

  // setup web handlers
  server.on("/", handleRoot);
  server.on("/dashboard", HTTP_GET, handleDashboard);
  server.on("/toggleLED", HTTP_GET, handleToggleLED);
  server.on("/toggleFan", HTTP_GET, handleToggleFan);
  server.on("/toggleAP1", HTTP_GET, handleToggleAP1);
  server.on("/toggleAP2", HTTP_GET, handleToggleAP2);
  server.on("/winOpen", HTTP_GET, handleWinOpen);
  server.on("/winClose", HTTP_GET, handleWinClose);
  server.on("/winStop", HTTP_GET, handleWinStop);
  server.on("/setPasswords", HTTP_POST, handleSetPasswords);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.printf("HTTP server started on port %d\n", WEB_PORT);

  // initialize baseline ultrasonic with a few reads
  baselineDistance = -1;
  for (int i = 0; i < 5; i++) {
    long d = ultrasonicReadCM();
    if (d > 0) {
      if (baselineDistance < 0) baselineDistance = d;
      else baselineDistance = (baselineDistance + d) / 2;
    }
    delay(100);
  }
  if (baselineDistance < 0) baselineDistance = 100; // default
  Serial.printf("Ultrasonic baseline set to %d cm\n", baselineDistance);
}

// ---------------------- MAIN LOOP ------------------------- //
void loop() {
  server.handleClient();

  // gesture handling
  gestureTask();

  // window safety enforcement
  windowSafetyTask();

  // If window action is active, stop after a small delay if manual stop requested or action end
  // Here we stop automatically after WINDOW_MAX_RUN_MS (handled in windowSafetyTask)

  // You can add scheduled actions here (e.g., turn on fan at certain times)
  // For example (pseudo-schedule) - left as placeholder for extension

  // short delay to keep loop responsive but not burning CPU
  delay(10);
}
