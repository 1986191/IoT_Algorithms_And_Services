#include <Arduino.h>

const int adcPin = 7;   // ADC1 (GPIO 1)
const uint16_t samples = 64; 

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); 
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation (default 0dB)
}

void loop() {
  unsigned long t = 0;
  unsigned long current_t;
  unsigned long last_t;

  for(int i=0; i<samples; i++)
   {
      last_t = micros();
      analogRead(adcPin);
      current_t = micros();
      t += current_t - last_t;
  }

  Serial.print("Frequency:");
  Serial.print(1000.f / (t / samples));
  Serial.println("khz");
}

/*
 * 33.33 khz 
 */