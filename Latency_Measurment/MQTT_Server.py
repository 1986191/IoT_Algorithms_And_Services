import paho.mqtt.client as mqtt

# MQTT broker settings
broker = "test.mosquitto.org"  # Change to your broker address
port = 1883

last_message = None  # Store the last received message to avoid duplicate sends

# Callback when a message is received
def on_message(client, userdata, msg):
    global last_message

    if msg.payload != last_message:  # Send only if it's a new message
        last_message = msg.payload   # Update last message

        # Ensure the payload is exactly 50 bytes
        response_message = msg.payload #.ljust(50, b' ')[:50]

        client.publish("1986191/window/callback", response_message)
        print(f"Sent message: {response_message}")

# Create an MQTT client
client = mqtt.Client()

# Assign the on_message callback
client.on_message = on_message

# Connect to the broker
client.connect(broker, port, 60)

# Subscribe to a topic
client.subscribe("1986191/window/average")

# Loop forever to process messages
client.loop_forever()
