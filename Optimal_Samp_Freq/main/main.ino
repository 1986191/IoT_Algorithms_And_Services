#include <Arduino.h>
#include "arduinoFFT.h"

const int adcPin = 7;
const uint16_t samples = 64;            // Must be a power of 2
const double samplingFrequency = 12.0;  // Define sampling frequency -> initially very high for oversampling

double vReal[samples];
double vImag[samples];

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);       // ADC bit resolution
  analogSetAttenuation(ADC_11db); // Set ADC attenuation
}

void loop() {
  // 1. Collect Samples
  for (int i = 0; i < samples; i++) {
    vReal[i] = analogRead(adcPin);
    vImag[i] = 0.0;                                     // Imaginary part always zero
    delayMicroseconds(1000000 / samplingFrequency);
  }

  // 2. Preprocessing -> get sliding window -> go to frequency domain -> numbers to signal power of each frequency f
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  
  // 3. Get Frequency Peak
  double peakFrequency = FFT.majorPeak();

  // 4. Print Peak Frequency
  Serial.println(peakFrequency, 6);

  // Wait
  delay(1000); 
}
