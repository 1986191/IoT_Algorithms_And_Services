# Sending Sliding Window Average using MQTT

#### Request:

>  Transmit the aggregate value, i.e. the average, to a nearby edge server using MQTT over WIFI.

---

#### Solution:

The code is based on the previous **Sliding Window Average** implementation, with added **WiFi** and **MQTT** connectivity functionality. 

The main differences, apart from the WiFi setup and MQTT client configuration, is that the average is now published to an MQTT topic instead of being printed on the serial console.

Functionality was tested by installing **Termux** on an Android device and running the following command:

> `mosquitto_sub -h test.mosquitto.org -t "topic_here"`

The received values were then verified to match the values computed by the sending device.
