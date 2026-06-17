#include "HUSKYLENS.h"
#include <SoftwareSerial.h>

HUSKYLENS huskylens;
SoftwareSerial huskySerial(2, 3); // RX=2, TX=3

// Define the pin connected to the relay
const int RELAY_PIN = 8; 

void setup() {
  Serial.begin(9600);
  huskySerial.begin(9600);
  
  // 1. Set the relay pin as an output
  pinMode(RELAY_PIN, OUTPUT);
  
  // 2. ACTIVE LOW: Start with the relay turned ON (LOW)
  digitalWrite(RELAY_PIN, LOW);
  
  // Initialize the HuskyLens
  while (!huskylens.begin(huskySerial)) {
    Serial.println("HuskyLens not found! Check wiring and protocol settings.");
    delay(100);
  }
  
  // Switch to Face Recognition mode
  huskylens.writeAlgorithm(ALGORITHM_FACE_RECOGNITION);
  Serial.println("HuskyLens Ready! Relay is currently ON.");
}

void loop() {
  if (!huskylens.request()) {
    // If connection drops, fail-safe to keep the relay ON
    digitalWrite(RELAY_PIN, LOW); 
  } 
  else if (huskylens.available()) {
    Serial.println("FACE DETECTED !! Turning OFF disinfection mode for 10 seconds...");
    
    // ACTIVE LOW: Send HIGH to turn the relay OFF
    digitalWrite(RELAY_PIN, HIGH);
    
    // Pause the Arduino completely for 10 seconds (10,000 milliseconds)
    delay(10000); 
    
    Serial.println("10 seconds over. Turning Relay back ON.");
    
    // ACTIVE LOW: Send LOW to turn the relay back ON
    digitalWrite(RELAY_PIN, LOW);
  } 
  else {
    // No face in view, ensure the relay stays ON
    digitalWrite(RELAY_PIN, LOW);
  }
  
  // A small delay before checking the camera again
  delay(100); 
}