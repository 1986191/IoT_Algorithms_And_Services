/*
 * The commented code is the oversampling code with theoretically no added delays
 * The uncommented code is the same as the commented one, but adding FreeRTOS for managing the sampling and performance evaluation Tasks
 */

/*
#include <Arduino.h>

const int adcPin = 7; // ADC input pin

void setup() 
{
    Serial.begin(115200);
}

void loop() 
{
    int sample = analogRead(adcPin);
    Serial.println(sample);
}*/

#include <Arduino.h>

const int adcPin = 7; // ADC input pin

TaskHandle_t MonitorTaskHandle = NULL;

void SamplingTask(void *parameter);
void MonitorTask(void *parameter);

void setup() 
{
    Serial.begin(115200);
    Serial.println("Starting Basic ADC Sampling with FreeRTOS...");

    xTaskCreatePinnedToCore(SamplingTask, "SamplingTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(MonitorTask, "MonitorTask", 4096, NULL, 1, &MonitorTaskHandle, 1);
}

void SamplingTask(void *parameter) 
{
    while (1) 
    {
        uint32_t startTime = micros();

        int sample = analogRead(adcPin);
        Serial.println(sample);

        uint32_t elapsedTime = micros() - startTime;

        Serial.print("[Sampling] Execution Time: ");
        Serial.print(elapsedTime);
        Serial.println(" µs");

        // Prevent watchdog reset - happening without this instructions on average once per second by resetting the device
        vTaskDelay(1);
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

        Serial.print("Task Stack High Water Mark: ");
        Serial.println(uxHighWaterMark);

        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Monitor every 2s
    }
}

void loop() {
    // Nothing here
}
