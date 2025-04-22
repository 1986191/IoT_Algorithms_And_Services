# Optimal Sampling Frequency

#### Request:

>Compute the FFT and adapt the sampling frequency accordingly. For example, for a maximum frequency of 5 Hz adapt the  sampling frequency to 10Hz.

---

#### Solution:

The code samples an input signal via the ADC and forwards the data to a Fast Fourier Transform (FFT) function.

The initial approach was to oversample the signal, using a sample rate of **10 kHz**, and then analyze the FFT results by identifying the furthest peak from the origin using the `FFT.majorPeak()` function. 

A major challenge with this approach was the presence of noise and the lack of precision in both the DAC and ADC at high frequencies. This resulted in varying peak frequencies being calculated at different times. To mitigate this issue, the devices were placed closer together and their grounds were shared.

For a composite signal of the form:

> 2sin(2π3t) + 4sin(2π5t)

the resulting major peak was detected at 6 Hz. As per the Nyquist sampling theorem, the final sampling frequency was set to 12 Hz.

FreeRTOS has not been used due to the simplicity of the request.

# Updated description

This code implements part of a multitasking data acquisition and processing system for the ESP32, using FreeRTOS for task scheduling and concurrency management. The system is designed to continuously sample, process, and monitor sensor data.

The SamplingTask is responsible for periodically reading analog data from an ADC pin at an adaptive sampling frequency. It manages the reading frequency by adjusting it based on a recalculated sampling frequency (recomputedSamplingFrequency). The task uses a semaphore to ensure safe, atomic sampling, preventing multiple tasks from accessing the buffer simultaneously and potentially causing race conditions ( for example due to buffer swaps) . It samples a predefined number of values, adjusting the delay between readings based on the current sampling frequency. After the sampling is complete, it notifies other tasks, such as the timer trigger and outliers check. The task also handles performance evaluation and manages the watchdog timer to avoid resets.

The ProcessingTask is responsible for analyzing sampled data to compute its peak frequency. Upon activation, it measures its own execution time for diagnostic purposes and calls the function ComputePeakFrequency_Arduino() to identify the dominant frequency in the current buffer of samples. If a valid frequency is detected (greater than 0), it applies the Nyquist theorem to calculate a new recommended sampling frequency — which is twice the detected peak frequency. To ensure hardware stability and reasonable data rates, this new sampling frequency is then constrained between 10 Hz and 33,000 Hz. This recalculated frequency is stored in a shared variable recomputedSamplingFrequency, which other tasks (just the sampling task) uses to adjust future data collection. If no valid frequency is found, a warning is printed to the Serial console. After the computation, the task logs its total execution time and resets the bufferState to BUFFER_EMPTY, signaling that the buffer is ready for new data. Finally, the task self-destructs using vTaskDelete(NULL) to free its stack and system resources. A thing to remember and that justifies this choice is that this function should be executed rarely.

The MonitorTask is designed to run indefinitely and serves as a system health monitor. It executes in a loop with a 2-second delay between iterations (defined by MONITORING_TIMER). During each cycle, it retrieves two key pieces of system information: the available heap memory (esp_get_free_heap_size()) and the "high water mark" of its own stack (uxTaskGetStackHighWaterMark()), which indicates how much stack space remains unused. Both metrics are printed to the Serial console, along with the current samplingFrequency. If an OLED display is enabled via the OLED_ENABLE macro, the same performance data is formatted and displayed on-screen. Additionally, the task blinks an LED to give visual feedback that it is alive and running, toggling the LED on at the start and off at the end of each monitoring cycle. This task is valuable for diagnosing memory leaks, stack overflows, or other resource constraints on embedded systems like the ESP32.

The TimerTriggerTask is an event-driven task designed to periodically trigger new FFT computations. It begins by blocking indefinitely, waiting for a notification that new sample data is available (ulTaskNotifyTake). Once notified, the task attempts to swap the active sample buffer using SwapBuffers(). If the swap is successful, it prints a confirmation message and spawns a new ProcessingTask using xTaskCreatePinnedToCore. The new task is pinned to core 1 and assigned a moderate priority of 2, ensuring the system's real-time behavior remains balanced. After launching the ProcessingTask, the TimerTriggerTask waits for a fixed period defined by FFT_TIMER before listening for the next notification. This structure decouples the timing of data sampling from the computationally expensive FFT analysis, allowing the system to remain reactive and stable under varying loads.

The other functions are simplier and better described in the code's comments.
