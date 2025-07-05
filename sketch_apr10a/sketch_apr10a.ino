#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Wi-Fi credentials
const char* ssid = "Tenda_2E4E00";      // Replace with your Wi-Fi SSID
const char* password = "ungureanu@16";  // Replace with your Wi-Fi password

// Web server on port 80
ESP8266WebServer server(80);

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  Serial.println("\nESP8266 Wi-Fi Test");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Define test endpoint
  server.on("/test", HTTP_GET, []() {
    Serial.println("Received test request");
    server.send(200, "text/plain", "ESP8266 is online");
  });
  server.begin();
  Serial.println("Web server started at /test");
}

void loop() {
  server.handleClient();  // Handle incoming HTTP requests
}