#include <Servo.h>

Servo myServo;
const int servoPin = 9;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.write(90);
  Serial.println("Arduino este gata. Se asteapta comenzi");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "SERVO_ON") {
      myServo.write(0);
      Serial.println("Servo ON");
    } else if (command == "SERVO_OFF") {
      myServo.write(90);
      Serial.println("Servo OFF");
    } else {
      Serial.println("Comanda necunoscuta");
    }
  }
}