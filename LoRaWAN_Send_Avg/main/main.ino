#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "LoRaWan_APP.h"

#define SENSOR_PIN        7           // ADC pin
#define SAMPLE_INTERVAL   100         // Sampling every 100ms
#define WINDOW_SIZE       50          // 5s window (50 samples if 100ms per sample -> 50 * 100ms = 5s)

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xF9, 0x16 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x28, 0xA1, 0x46, 0x74, 0xC7, 0x8A, 0x75, 0xE0, 0xC2, 0xF0, 0x8C, 0xC1, 0xDB, 0x86, 0xAE, 0xA9 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x03, 0x59, 0x9D, 0x1E, 0x47, 0x6C, 0x42, 0xFE, 0xD9, 0x65, 0xEA, 0x10, 0xC4, 0xB9, 0x0D, 0xF1 };
uint8_t appSKey[] = { 0x71, 0x7D, 0xB7, 0x77, 0x18, 0x26, 0xE4, 0x32, 0x53, 0x56, 0x44, 0xE5, 0x41, 0x32, 0x97, 0x2D };
uint32_t devAddr =  ( uint32_t )0x260B627F;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_C;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

/*  Number of trials when no aknowledgment */
uint8_t confirmedNbTrials = 4;

/* FreeRTOS */
QueueHandle_t dataQueue;

/* Task handlers */
TaskHandle_t loRaTaskHandle = NULL;
TaskHandle_t dataCollectionTaskHandle = NULL;
TaskHandle_t computeAverageTaskHandle = NULL;

/* Prepares the payload of the frame */
static void PrepareTxFrame( uint8_t port, float average )
{
    appDataSize = sizeof(float);  // Sending 4 bytes
    memcpy(appData, &average, sizeof(float));  // Copy float into payload
}


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

            // Send average once every 5 seconds
            if (index == 0) 
            {
                float average = sum / WINDOW_SIZE;
                Serial.print("Avg: ");
                Serial.println(average, 3);
                xTaskNotify(loRaTaskHandle, *(uint32_t*)&average, eSetValueWithOverwrite); // Correctly notify with value
            }
        }
    }
}

/* https://github.com/HelTecAutomation/Heltec_ESP32/blob/master/examples/LoRaWAN/LoRaWan/LoRaWan.ino */
void LoRaTask(void *pvParameters)
{
    while (1)  // Ensure the task keeps running
    {
        switch (deviceState) 
        {
            case DEVICE_STATE_INIT:
            {
                #if (LORAWAN_DEVEUI_AUTO)
                    LoRaWAN.generateDeveuiByChipID();
                #endif
                LoRaWAN.init(loraWanClass, loraWanRegion);
                LoRaWAN.setDefaultDR(3);
                deviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                LoRaWAN.join();
                deviceState = DEVICE_STATE_SEND;
                break;
            }
            case DEVICE_STATE_SEND:
            {
                Serial.println("Preparing to send data...");
                PrepareTxFrame(appPort, 0.f);
                LoRaWAN.send();
                Serial.println("Data sent!");
                deviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                // Schedule next packet
                txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
                vTaskDelay(pdMS_TO_TICKS(txDutyCycleTime));
                deviceState = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                Serial.println("Waiting for new data...");
                uint32_t receivedValue;
                if (xTaskNotifyWait(0, 0, &receivedValue, portMAX_DELAY))  // Correct way to receive value
                {
                    float average = *(float*)&receivedValue;
                    PrepareTxFrame(appPort, average);
                    deviceState = DEVICE_STATE_SEND;  // Now send after waking up
                    Serial.println("Data sent...");
                }
                break;
            }
            default:
            {
                deviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}

void setup() 
{
    Serial.begin(115200);
    analogReadResolution(10);
    analogSetAttenuation(ADC_11db);

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);   // LoRa Requirement

    dataQueue = xQueueCreate(WINDOW_SIZE, sizeof(float));

    xTaskCreatePinnedToCore(LoRaTask, "LoRaTask", 8192, NULL, 1, &loRaTaskHandle, 0);

    while(deviceState != NULL && (deviceState == DEVICE_STATE_INIT || deviceState == DEVICE_STATE_JOIN)) { vTaskDelay(pdMS_TO_TICKS(100)); /* Busy wait for connection */ }

    xTaskCreatePinnedToCore(DataCollectionTask, "DataCollectionTask", 2048, NULL, 2, &dataCollectionTaskHandle, 1);
    xTaskCreatePinnedToCore(ComputeAverageTask, "ComputeAverageTask", 2048, NULL, 3, &computeAverageTaskHandle, 1);
}

void loop() 
{
    vTaskDelay(portMAX_DELAY);
}
