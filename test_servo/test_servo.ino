#include <Servo.h>

Servo myServo;           
const int servoPin = 9;     
int initialPos = 85;         
int openPos = 65;        

void setup() {
  myServo.attach(servoPin);    
  myServo.write(initialPos);    
  Serial.begin(9600);           
}

void loop() {
  if (Serial.available() > 0) {   
    char command = Serial.read(); 
    if (command == '1') {         
      myServo.write(openPos);   
      delay(500);                 
      myServo.write(initialPos); 
    }
  }
}