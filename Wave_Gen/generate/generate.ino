#include <Arduino.h>

// Define the DAC output pin
const int dacPin = 25;  // DAC1 (GPIO 25)

// Signal parameters
const float freq1 = 3.0;  // Frequency of first sine wave (Hz)
const float freq2 = 5.0;  // Frequency of second sine wave (Hz)
const int amplitude1 = 2;
const int amplitude2 = 4;
const int offset = 128;

// Sampling parameters
const int samplingFrequency = 100;
const int samples = 128;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
}

void loop() {
  for (int i = 0; i < samples; i++) {
    float t = (float)i / samplingFrequency;  // Time step

    // Generate wave
    int sineValue = offset + (amplitude1 * sin(2.0 * PI * freq1 * t)) + (amplitude2 * sin(2.0 * PI * freq2 * t));

    // Output to DAC
    dacWrite(dacPin, sineValue);

    // Plot on serial
    Serial.println(sineValue);

    // Wait
    delayMicroseconds(1000000 / samplingFrequency);
  }
}
