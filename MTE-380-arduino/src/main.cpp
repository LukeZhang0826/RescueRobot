#include <Arduino.h>

unsigned long blinkDelay = 100; // ms
unsigned long lastToggle = 0;
bool ledState = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // --- handle serial input ---
  if (Serial.available()) {
    long val = Serial.parseInt();   // read number from serial
    if (val > 0 && val <= 5000) {   // sanity bounds
      blinkDelay = val;
      Serial.print("Blink delay set to ");
      Serial.println(blinkDelay);
    }
  }

  // --- non-blocking blink ---
  unsigned long now = millis();
  if (now - lastToggle >= blinkDelay) {
    lastToggle = now;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
}
