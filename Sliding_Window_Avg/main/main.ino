#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define SENSOR_PIN        7           // ADC pin
#define SAMPLE_INTERVAL   100         // Sampling every 100ms
#define WINDOW_SIZE       50          // 5s window (50 samples if 100ms per sample -> 50 * 100ms = 5s)

QueueHandle_t dataQueue;

// Data Collection Task
void DataCollectionTask(void *pvParameters) 
{
    float sensorValue;
    while (1) 
    {
        sensorValue = analogRead(SENSOR_PIN);
        xQueueSend(dataQueue, &sensorValue, portMAX_DELAY);   // Send data to queue
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL));           // Wait 100ms
    }
}

// Average the data task
void ComputeAverageTask(void *pvParameters) 
{
    float slidingWindow[WINDOW_SIZE] = {0};
    float currentValue = 0;
    float sum = 0;
    int index = 0;

    while (1) 
    {
        // Wait for new data
        if (xQueueReceive(dataQueue, &currentValue, portMAX_DELAY) == pdPASS) 
        {
            // Subtract the old value from sum
            sum -= slidingWindow[index];

            // Insert new value into sliding window
            slidingWindow[index] = currentValue;
            sum += currentValue;
            
            // Update the index (circular buffer)
            index = (index + 1) % WINDOW_SIZE;

            // Compute average
            float average = sum / WINDOW_SIZE;
            Serial.print("Avg: ");
            Serial.println(average, 3);

            // Once every 5 seconds, calculate and print the average -> useful for further requests like sending data not too often
            /*
            if (index == 0) 
            {
                float average = sum / WINDOW_SIZE;
                Serial.print("Avg: ");
                Serial.println(average, 3);
            }
            */
        }
    }
}

void setup() 
{
    Serial.begin(115200);
    analogReadResolution(10);
    analogSetAttenuation(ADC_11db);

    dataQueue = xQueueCreate(WINDOW_SIZE, sizeof(float));

    xTaskCreatePinnedToCore(DataCollectionTask, "DataCollectionTask", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(ComputeAverageTask, "ComputeAverageTask", 2048, NULL, 2, NULL, 1);
}

void loop() 
{
    vTaskDelay(portMAX_DELAY);
}
