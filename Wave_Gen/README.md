# Wave Generator

**A better version of the code can be found inside the */Bonus/Wave_Generator/* folder**

---

#### Scope:
This code generates a composite wave signal of the form:

> 2sin(2π3t) + 4sin(2π5t)

The signal combines two sine waves of different frequencies, 3Hz and 5Hz, with amplitudes of 2 and 4, respectively.

---

#### Hardware Information and Setup:
The wave is generated using an **ESP32 WRoom Dev Kit**.

The output signal is transmitted via the **DAC pin 25**, which may present some noise.

For better reading accuracy, the ground pins of both the wave-generating device and the wave-sampling device have been connected.
