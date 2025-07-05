#include <WiFi.h>
#include <WebServer.h>

// Wi-Fi credentials
const char* ssid = "Tenda_2E4E00";     
const char* password = "ungureanu@16";

// Static IP configuration
IPAddress staticIP(192, 168, 0, 101);  // Static IP for ESP32
IPAddress gateway(192, 168, 0, 1);     // Gateway
IPAddress subnet(255, 255, 255, 0);     // Subnet mask

// Web server on port 80
WebServer server(80);

// LED pin (onboard LED for most ESP32 boards is GPIO 2)
const int ledPin = 2;

// Relay pin for pump control (changed to GPIO 18)
const int relayPin = 18;
bool pumpState = false;  // Pump state (false = off, true = on)

void blinkLED(int blinks, int delayTime) {
  for (int i = 0; i < blinks; i++) {
    digitalWrite(ledPin, HIGH);
    delay(delayTime);
    digitalWrite(ledPin, LOW);
    delay(delayTime);
  }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("Booting...");

  // Immediately set relay pin to OFF state (active-low relay: HIGH = OFF)
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);  // Ensure pump is OFF at boot
  Serial.println("Relay pin set HIGH (pump OFF)");

  // Initialize LED pin (reflects pump state)
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // LED off (pump off)
  Serial.println("LED pin set LOW (pump OFF)");

  // Configure static IP
  if (!WiFi.config(staticIP, gateway, subnet)) {
    Serial.println("Static IP configuration failed!");
    while (true) {
      blinkLED(5, 100);  // Rapid blink: 5 blinks, 100ms each
      delay(1000);
    }
  }

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    blinkLED(2, 500);  // Blink LED slowly while connecting (2 blinks, 500ms each)
    delay(1000);
    Serial.println("Still connecting...");
  }
  Serial.println("Wi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Define HTTP endpoints
  server.on("/toggle", HTTP_GET, []() {
    pumpState = !pumpState;
    digitalWrite(relayPin, pumpState ? LOW : HIGH);  // Active-low: LOW = ON, HIGH = OFF
    digitalWrite(ledPin, pumpState ? HIGH : LOW);  // LED reflects pump state
    Serial.println("Toggle endpoint called: Pump " + String(pumpState ? "ON" : "OFF"));
    server.send(200, "text/plain", "LED toggled to " + String(pumpState ? "ON" : "OFF"));
  });

  server.on("/signal", HTTP_GET, []() {
    pumpState = true;  // Turn pump on
    digitalWrite(relayPin, LOW);  // Relay active-low: LOW = ON
    digitalWrite(ledPin, HIGH);  // LED on (pump on)
    Serial.println("Signal endpoint called: Pump ON");
    server.send(200, "text/plain", "Pump turned ON");
  });

  server.on("/off", HTTP_GET, []() {
    pumpState = false;  // Turn pump off
    digitalWrite(relayPin, HIGH);  // Relay active-low: HIGH = OFF
    digitalWrite(ledPin, LOW);  // LED off (pump off)
    Serial.println("Off endpoint called: Pump OFF");
    server.send(200, "text/plain", "Pump turned OFF");
  });

  // Start the server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();  // Handle incoming HTTP requests
  delay(10);  // Small delay to prevent watchdog timer reset
}