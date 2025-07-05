#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "CB006biss"; 
const char* password = "vreausainvat";

IPAddress staticIP(192, 168, 5, 101);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

const int ledPin = 2;

const int relayPin = 18;
bool pumpState = false;

void connectToWiFi() {
  Serial.println("Se conecteaza la WI-FI...");
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    blinkLED(2, 500);
    Serial.println("Se asteapta conexiunea...");
    delay(1000);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi conectat!");
    Serial.print("Adresa IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Nu s-a reusit conectarea!");
  }
}

void blinkLED(int blinks, int delayTime) {
  for (int i = 0; i < blinks; i++) {
    digitalWrite(ledPin, HIGH);
    delay(delayTime);
    digitalWrite(ledPin, LOW);
    delay(delayTime);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  Serial.println("Pinul releului setat la low (pompa inchisa)");

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  Serial.println("Pinul led setat la low (pompa deschisa)");

  if (!WiFi.config(staticIP, gateway, subnet)) {
    Serial.println("IP static configurare esuata!");
    while (true) {
      blinkLED(5, 100);
      delay(1000);
    }
  }

  connectToWiFi();

  server.on("/toggle", HTTP_GET, []() {
    pumpState = !pumpState;
    digitalWrite(relayPin, pumpState ? HIGH : LOW);
    digitalWrite(ledPin, pumpState ? HIGH : LOW);
    Serial.println("Toggle endpoint called: Pump " + String(pumpState ? "ON" : "OFF"));
    server.send(200, "text/plain", "LED toggled to " + String(pumpState ? "ON" : "OFF"));
  });

  server.on("/signal", HTTP_GET, []() {
    pumpState = true;
    digitalWrite(relayPin, HIGH);
    digitalWrite(ledPin, HIGH);
    Serial.println("Signal endpoint called: Pump ON");
    server.send(200, "text/plain", "Pump turned ON");
  });

  server.on("/off", HTTP_GET, []() {
    pumpState = false;
    digitalWrite(relayPin, LOW);
    digitalWrite(ledPin, LOW);
    Serial.println("Off endpoint called: Pump OFF");
    server.send(200, "text/plain", "Pump turned OFF");
  });

  server.begin();
  Serial.println("HTTP server pornit");
}

void loop() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= 5000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wi-Fi deconectat!");
      connectToWiFi();
      if (WiFi.status() == WL_CONNECTED) {
        server.begin();
        Serial.println("HTTP restart");
      }
    } else {
      long rssi = WiFi.RSSI();
      Serial.print("Wi-Fi RSSI: ");
      Serial.print(rssi);
      Serial.println(" dBm");
    }
    lastCheck = millis();
  }

  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  delay(10);
}