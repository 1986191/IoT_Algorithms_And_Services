# Maximum sampling frequency

#### Request:

>Identify the maximum sampling frequency of your hardware device, for example, 100Hz. Note 100Hz is only an example. You need to demonstrate the ability of sampling at a very high frequency.

---

#### Solution:

The main objective of this code was to sample the ADC as fast as possible by avoiding any delays within the `loop()` cycle. By doing so, the loop iterations and ADC readings are constrained only by the CPU cycles. The total time needed for sampling is computed and then averaged as in the following snipset:

```cpp
for (int i = 0; i < samples; i++) {
    last_t = micros();
    analogRead(adcPin);
    current_t = micros();
    t += current_t - last_t;
}
Serial.print(1000.f / (t / samples));
```

Where **t** is the total time calculated for **samples** samples, and **t/samples** is the average time. The value printed on the serial monitor will be the frequency in khz. 

**Notice** that **FreeRTOS** was intentionally not used to avoid any potential delays introduced by the operating system.

Testing was performed using one device as a signal generator through its DAC (refer to the Wave Generator), and an **Heltec WiFi LoRa (V3)** as the sampling device. 

The maximum achieved sampling rate during testing was **`33.33 kHz`**.