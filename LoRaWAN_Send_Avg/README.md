# Send Sliding Window Average through LoRaWAN

#### Request:

> Transmit the aggregate value, i.e. the average, to a cloud server using LoRaWAN + TTN.

---

#### Solution:

Following the example provided by Heltec in the [Heltec Repository](https://github.com/HelTecAutomation/Heltec_ESP32/blob/master/examples/LoRaWAN/LoRaWan/LoRaWan.ino), LoRaWAN connectivity has been integrated into the project.  

##### Device Registration on MTT  
The device was registered on **MTT (The Things Network)** by selecting the closest available device type. Since the **Heltec LoRa V3** option was not available, the device type was set to **Heltec LoRa V2**. 
The following configurations were applied during registration:  
- **Device Type:** Heltec LoRa V2  
- **LoRaWAN Region:** Europe (using EU868 frequency plan)  
- **Data Rate Range:** Set to 9, for optimal balance between range and data rate.

##### Configuration in the Code  
Before running the device, the necessary credentials and configurations were properly set in the firmware:  
- **EUI (Extended Unique Identifier):** The unique device identifier was retrieved and correctly set in the source code.  
- **Application Secure Key (AppKey):** The key was copied from MTT and inserted into the firmware for secure authentication.  
- **Network Session Key (NwkSKey) and Application Session Key (AppSKey):** These were generated or retrieved from MTT and configured for secure communication.  
- **OTAA (Over-the-Air Activation):** The device was configured to use **OTAA** for better security and ease of management.
- **ABP (Activation by Personalization):** Altough present, not used due to deprecation and lack of security.

With these configurations, the device was successfully prepared to establish a LoRaWAN connection and communicate with the network server the computed sliding window average.
