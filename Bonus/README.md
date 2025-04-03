# Bonus assignment

#### Request:

> Consider at least 3 different input signals and measure the performance of the system. Discuss different types of an input signal may affect the overall performance in the case of adaptive sampling vs basic/over-sampling.

---

#### Overview:

The assignment will be tested by generating different sinusoidal signals using the ESP32 WRoom DAC as in the previous assignments. Then performance differencies between oversampling and adaptive sampling will be evaluated by measuring how the different inputs affect the performance, by measuring for example CPU usage, memory and energy consumption.

##### Input signals:
The signals that will be generated will be a combination of:
- Low frequency and high amplitude signal 
- High frequency and low amplitude signal
- A mix of the above properties

So the generated signal could be something in the form:
> 8sin(2π 50t) + 2sin(2π 2000t) + 4sin(2π 800t)

##### Performance metrics:

- *Sampling rate efficiency*: how many samples are acquired for each unit of time and how many of them could be avoided by optimal sampling.
- *Memory usage*: how much memory is required to store the sampling, how adaptive sampling reduces memory usage.
- *CPU usage*: how much of the CPU is used when oversampling compared to optimal sampling, and how much processing time is required for the Fourier Transformation processing.
- *FFT Accuracy*: how the samplnig rate affects the FFT resolution and if oversampling provides unnecessary data.
- *Power consumption*: already addressed in previous assignments, how the power consumption scales with different input signals.
- *Overall performance scaling*: what to expect when the input signal is simplier or more complex.

--- 

##### Performance evaluation:

Oversampling serial console example output (from real experimentation):

> ------ Performance Metrics ------ \
> Free Heap Memory: 348420 bytes \
> Task Stack High Water Mark: 2944 \
> 2023 \
> [Sampling] Execution Time: 10590 µs \
> 2060 \
> [Sampling] Execution Time: 60 µs \
> 2035 \
> [Sampling] Execution Time: 60 µs \
> 2037 \
> [Sampling] Execution Time: 60 µs \
> 1909 \
> [Sampling] Execution Time: 60 µs \
> 1927 

Adaptive sampled serial console example output (from real experimentation):

> ------ Performance Metrics ------ \
> Free Heap Memory: 342860 bytes \
> Sampling Frequency: 1068.15 Hz \
> Task Stack High Water Mark: 3032 \
> [Sampling] Execution Time: 61.97 ms \
> [Processing] Peak Frequency: 8000.00 \
> [Processing] Execution Time: 1.49 ms \
> [Sampling] Execution Time: 7.06 ms \
> [Processing] Peak Frequency: 8039.35 \
> [Processing] Execution Time: 1.49 ms \
> [Sampling] Execution Time: 6.16 ms 

|**Metric**      | **Ovesampled** | **Adaptive sampled**|
|----------------|----------------|---------------------|
|**Power**       |47mA            |89mA                 |
|**Peak freq**   |16.6kHz         |8kHz                 |
|**Task Stack**  |2944            |3032                 |
| **Free Heap Memory**              | 348420 bytes                                  | 342860 bytes                                                              |
| **Sampling Execution Time**       | 60µs (mostly) with occasional spikes          | Ranges from 7.06ms to 61.97ms                                             |
| **Processing Execution Time**     | Not applicable (FFT not used)                 | 1.49ms                                                                    |
| **Sampling Frequency**            | Fixed at high rate                            | Adaptive (varies, ~1068Hz measured)                                       |
| **Memory Utilization**            | More free memory (less processing overhead)   | More memory usage due to FFT & adaptive logic                             |
| **CPU Load**                      | Lower, but inefficient usage due to constant high-rate sampling | Higher, but more efficient due to adaptive adjustments  |
| **FFT Precision**                 | Not applicable | More accurate due to properly chosen Nyquist sampling                                                    |
| **Data Redundancy**               | High (many unnecessary samples)               | Reduced (only meaningful samples captured)                                |
| **Duty Cycle**                    | High (constant high-rate sampling)            | Lower (sampling adjusts dynamically)                                      |


*What is Task Stack High Water Mark?* 
uxTaskGetStackHighWaterMark() Copy returns the minimum amount of remaining stack space that was available to the task since the task started executing.

---

##### Observations:
- Oversampling works very well for very high frequency signals, but becomes wasteful for low frequency ones. Also depending on the signal-generator source, when oversampling a low frequency signal there might be some difficulties in reconstructing the signal due to the signal-generator not changing the read value fast enough, thous resulting in the possibilty of observing steps signals insted of the proper generated one. This was the case with low frequency signals using the ESP32 DAC.
- Adaptive sampling is more efficient under different aspects as stated above, but in case of sharp signal transitions, it could possibly lead to lower FFT accuracy. Overall the benefits are a lower CPU load and an improved battery life, leaving room for other tasks or computations that might be needed to be performed on the same device.
- The more signals there are, the more memory is needed, adaptive sampling helps reducing this problem. Due to the program, the variables, and many other data residing in memory, using too much RAM for sampling might be disruptive for the device, especially when the resources are limited (altough on the Heltec LoRa V3 they are more than sufficient).
- FFT requires uniform sampling, meaning that changing the sampling rate might disrupt the correct functioning and introduce artifacts. This issue might be mitigated by proper interpolation.
- The adaptive sampling code ends up using much more energy due to using both CPU cores, one for sampling and one for adapting the sample rate, ending up using up to more than double the oversampling example power without the performance metrics applied. But if FreeRTOS tasks are being used in both codes, they both end up using the same amount of energy.
- The oversampling sample code max frequency is much lower to the real max sampling frequency probably due to the use of FreeRTOS, proving that it introduces undesired delays.
