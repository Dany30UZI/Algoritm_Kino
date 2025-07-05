#include <ESP8266WiFi.h>
   #include <ESP8266WebServer.h>

   // Wi-Fi credentials
   const char* ssid = "CTANM6";      // Replace with the SSID of the 192.168.91.x network
   const char* password = "Vreausainvat2021";  // Replace with your Wi-Fi password

   // Web server on port 80
   ESP8266WebServer server(80);

   void setup() {
     // Initialize serial for debugging
     Serial.begin(115200);
     delay(1000);
     Serial.println("\nESP8266 Signal Test");

     // Set static IP (match your Wi-Fi subnet)
     IPAddress staticIP(192, 168, 91, 100);  // Updated to match Wi-Fi subnet
     IPAddress gateway(192, 168, 91, 1);     // Updated gateway
     IPAddress subnet(255, 255, 255, 0);
     WiFi.config(staticIP, gateway, subnet);

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

     // Define signal endpoint
     server.on("/signal", HTTP_GET, []() {
       Serial.println("Signal received!");
       server.send(200, "text/plain", "Signal received");
     });

     server.begin();
     Serial.println("Web server started. Endpoint: /signal");
   }

   void loop() {
     server.handleClient();  // Handle incoming HTTP requests
   }