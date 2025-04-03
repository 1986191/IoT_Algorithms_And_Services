#include <Arduino.h>
#include "arduinoFFT.h"

const int adcPin = 7; 
const uint16_t samples = 64;               // Must be a power of 2

// Initial Sampling Frequency (16 kHz) - oversampling at start
double samplingFrequency = 16000.0;  

// FFT Object
double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT(vReal, vImag, samples, samplingFrequency);

// Task Handles
TaskHandle_t SamplingTaskHandle = NULL;
TaskHandle_t ProcessingTaskHandle = NULL;
TaskHandle_t MonitorTaskHandle = NULL;

void setup() 
{
    Serial.begin(115200);
    Serial.println("Starting Adaptive Sampling with FreeRTOS...");

    analogReadResolution(10);
    analogSetAttenuation(ADC_11db);

    xTaskCreatePinnedToCore(SamplingTask, "SamplingTask", 4096, NULL, 1, &SamplingTaskHandle, 0);
    xTaskCreatePinnedToCore(ProcessingTask, "ProcessingTask", 4096, NULL, 2, &ProcessingTaskHandle, 1);
    xTaskCreatePinnedToCore(MonitorTask, "MonitorTask", 4096, NULL, 1, &MonitorTaskHandle, 1);
}

void SamplingTask(void *parameter) 
{
    while (1) 
    {
        uint32_t startTime = micros();

        for (int i = 0; i < samples; i++)
       {
            vReal[i] = analogRead(adcPin);  
            vImag[i] = 0.0;  // No imaginary component
            
            // Delay based on adaptive frequency
            delayMicroseconds(1000000 / samplingFrequency);
        }

        uint32_t elapsedTime = micros() - startTime;
        Serial.print("[Sampling] Execution Time: ");
        Serial.print(elapsedTime / 1000.0);
        Serial.println(" ms");

        // Notify FFT task that data is ready
        xTaskNotifyGive(ProcessingTaskHandle);

        vTaskDelay(1);  // Prevent watchdog reset
    }
}

void ProcessingTask(void *parameter) 
{
    while (1) 
    {
        // Wait until data is available
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        uint32_t startTime = micros();

        // Compute FFT - needed preprocessing -> get sliding window -> go to frequency domain -> numbers to signal power of each frequency f
        FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(FFT_FORWARD);
        FFT.complexToMagnitude();

        // Get max f
        double peakFrequency = FFT.majorPeak();
        Serial.print("[Processing] Peak Frequency: ");
        Serial.println(peakFrequency, 2);
        if (peakFrequency > 0) 
        {
            double newSamplingFreq = 2 * peakFrequency;

            // Constrain within limits (10 hz, max frequency found in other assignment)
            samplingFrequency = constrain(newSamplingFreq, 10.0, 33000.0);
        }

        uint32_t elapsedTime = micros() - startTime;
        Serial.print("[Processing] Execution Time: ");
        Serial.print(elapsedTime / 1000.0);
        Serial.println(" ms");

        vTaskDelay(1);  // Prevent watchdog reset - happening once per second without
    }
}

void MonitorTask(void *parameter) 
{
    while (1) 
    {
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        uint32_t freeHeap = esp_get_free_heap_size();

        Serial.println("------ Performance Metrics ------");
        Serial.print("Free Heap Memory: ");
        Serial.print(freeHeap);
        Serial.println(" bytes");

        Serial.print("Sampling Frequency: ");
        Serial.print(samplingFrequency);
        Serial.println(" Hz");

        Serial.print("Task Stack High Water Mark: ");
        Serial.println(uxHighWaterMark);

        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Monitor every 2s
    }
}

void loop() 
{
    // Nothing here again
}
