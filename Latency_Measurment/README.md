# Messages Latency

The goal of this code is to implement a simple server that forwards received messages on a topic that is also monitored by the sender. By measuring the Round Trip Time (RTT), we approximate the message latency as half of the RTT.

## Example Output

Below is an example output of the running code:


> WiFi_Connected
> 192.168.1.11
>48:CA:43:3B:4A:18
>48_CA_43_3B_4A_18
>Attempting MQTT connection...clientId-217 connected
>Publish message: 489.500000
>Message arrived on topic: 1986191/window/callback. Message: 489.500000
>Latency ms:
>55
>Publish message: 489.440002
>Message arrived on topic: 1986191/window/callback. Message: 489.440002
>Latency ms:
>61
>Publish message: 488.899994
>Message arrived on topic: 1986191/window/callback. Message: 488.899994
>Latency ms:
>72
>Publish message: 489.760010
>Message arrived on topic: 1986191/window/callback. Message: 489.760010
>Latency ms:
>77
>Publish message: 489.739990
>Message arrived on topic: 1986191/window/callback. Message: 489.739990
>Latency ms:
>88
>Publish message: 489.820007
>Message arrived on topic: 1986191/window/callback. Message: 489.820007
>Latency ms:
>98
>Publish message: 490.899994
>Message arrived on topic: 1986191/window/callback. Message: 490.899994
>Latency ms:
>44
>Publish message: 489.820007
>Message arrived on topic: 1986191/window/callback. Message: 489.820007
>Latency ms:
>39
>Publish message: 490.399994
>Message arrived on topic: 1986191/window/callback. Message: 490.399994
>Latency ms:
>70
