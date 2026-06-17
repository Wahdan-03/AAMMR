#include "HUSKYLENS.h"
#include <SoftwareSerial.h>


HUSKYLENS huskylens;


// RX = 2 , TX = 3
SoftwareSerial huskySerial(2, 3);


const char* names[] = {
  "Unknown no drawer for them",
  "Wahdan's drawer",
  "Meran's drawer",
  "Bouzo's drawer",
  "Maria's drawer",
  "Friend4"
};


// Lamp pins
int ledPins[] = {7, 9, 10, 11};


void setup() {


  Serial.begin(9600);
  huskySerial.begin(9600);


  Serial.println("Connecting to HuskyLens...");


  while (!huskylens.begin(huskySerial)) {
    Serial.println("HUSKYLENS not found!");
    delay(1000);
  }


  huskylens.writeAlgorithm(ALGORITHM_FACE_RECOGNITION);


  // Setup lamps
  for (int i = 0; i < 4; i++) {


    pinMode(ledPins[i], OUTPUT);


    // OFF at startup (ACTIVE-HIGH: LOW = OFF)
    digitalWrite(ledPins[i], LOW);
  }


  Serial.println("System Ready");
}


void loop() {


  // Turn OFF all lamps first (ACTIVE-HIGH: LOW = OFF)
  for (int i = 0; i < 4; i++) {
    digitalWrite(ledPins[i], LOW);
  }


  if (!huskylens.request()) {


    Serial.println("Communication failed!");
    delay(100);
    return;
  }


  int blocks = huskylens.count();


  if (blocks > 0) {


    Serial.print(blocks);
    Serial.println(" face(s) detected");


    for (int i = 0; i < blocks; i++) {


      HUSKYLENSResult result = huskylens.get(i);


      int id = result.ID;


      Serial.print("Detected ID: ");
      Serial.println(id);


      // ONLY turn ON lamp if ID is 1-4 (recognized faces)
      if (id >= 1 && id <= 4) {


        // ACTIVE-HIGH → HIGH turns lamp ON
        digitalWrite(ledPins[id - 1], HIGH);
        
        Serial.print("✓ Turning ON lamp for ");
        Serial.println(names[id]);
      }
      else if (id == 0) {
        Serial.println("✗ Unknown face - no lamp turned ON");
      }
      else {
        Serial.print("✗ ID ");
        Serial.print(id);
        Serial.println(" - no lamp assigned");
      }
    }
  }
  else {
    Serial.println("No faces detected");
  }


  delay(300);
}