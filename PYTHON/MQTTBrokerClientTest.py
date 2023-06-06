import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import time

'''
brew install mosquitto
/usr/local/sbin/mosquitto -c /usr/local/etc/mosquitto/mosquitto.conf
'''

broker_address = "localhost"
broker_port = 1883


import threading
def startBroker():
    exec(open("PYTHON/startBroker.py").read())
threading.Thread(target=startBroker).start()

# Callback triggered when the client connects to the MQTT broker
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    # Subscribe to a topic when connected
    client.subscribe("mytopic")

# Callback triggered when a message is received on a subscribed topic
def on_message(client, userdata, msg):
    print("Received message: " + str(msg.payload.decode()))

    # Publish a response message
    response = "Response message"
    client.publish("response_topic", response)

# Create a new MQTT client instance
client = mqtt.Client()

# Set the callback functions
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
client.connect(broker_address, broker_port)

# Start the MQTT network loop in the background
client.loop_start()

# Loop indefinitely
while True:
    try:
        # Publish a message
        message = "Hello, MQTT!"
        client.publish("mytopic", message)

        # Sleep for a while
        time.sleep(1)

    except KeyboardInterrupt:
        # Stop the MQTT network loop
        client.loop_stop()
        break

# Start the MQTT broker
publish.multiple([
    {
        'topic': 'mytopic',
        'payload': 'Hello from broker!',
        'qos': 0,
        'retain': False
    }
], hostname=broker_address, port=broker_port)
