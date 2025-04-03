#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// MQTT 
#include <WiFi.h>
#include "PubSubClient.h"

#include "secrets.h"

// Window
#define SENSOR_PIN        7           // ADC pin
#define SAMPLE_INTERVAL   100         // Sampling every 100ms
#define WINDOW_SIZE       50          // 5s window (50 samples if 100ms per sample -> 50 * 100ms = 5s)

// WiFi + MQTT setup
const char* ssid = SECRET_SSID;
const char* password = SECRET_PWD;
const char* mqttServer = "test.mosquitto.org";
int port = 1883;

String stMac;
long lastMsg = 0;
char mac[50];
char clientId[50];

#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];

WiFiClient espClient;
PubSubClient client(espClient);

QueueHandle_t dataQueue;

// WiFi connection
void WifiConnect() 
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
    }
}

void mqttReconnect() {
    while (!client.connected()) 
    {
      Serial.print("Attempting MQTT connection...");
      long r = random(1000);
      sprintf(clientId, "clientId-%ld", r);
      // to connect to adafruit
      // if (client.connect(clientId,SECRET_MQTT_USER, SECRET_MQTT_API_KEY)) {
      if (client.connect(clientId)) 
      {
        Serial.print(clientId);
        Serial.println(" connected");
        client.subscribe("avitaletti/feeds/threshold");
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        delay(5000);
      }
    }
}

// Publish message
void PublishMsg(float value)
{
    snprintf (msg, MSG_BUFFER_SIZE, "%lf", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    int result = client.publish("1986191/window/average", msg);
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

            float average = sum / WINDOW_SIZE;
            Serial.println(average);
            
            // Send average once every 5 seconds to be able to measure properly the RTT
            if (index == 0) 
            {
                PublishMsg(average);
            }
        }
    }
}

void SetupWiFi()
{
    randomSeed(analogRead(0));

    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WifiConnect();

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.macAddress());
    stMac = WiFi.macAddress();
    stMac.replace(":", "_");
    Serial.println(stMac);
    
    client.setServer(mqttServer, port);
    //client.setCallback(callback);
}

void setup() 
{
    Serial.begin(115200);
    analogReadResolution(10);
    analogSetAttenuation(ADC_11db);

    SetupWiFi();

    dataQueue = xQueueCreate(WINDOW_SIZE, sizeof(float));

    xTaskCreatePinnedToCore(DataCollectionTask, "DataCollectionTask", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(ComputeAverageTask, "ComputeAverageTask", 4096, NULL, 2, NULL, 1);
}

void loop() 
{
    if (!client.connected()) {
      mqttReconnect();
    }
    client.loop();

    vTaskDelay(portMAX_DELAY);
}
