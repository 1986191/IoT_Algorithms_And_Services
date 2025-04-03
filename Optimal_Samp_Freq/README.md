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