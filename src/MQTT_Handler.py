from paho.mqtt import client as mqtt_client
import time


class MQTTHandler:
    def __init__(self, client_id, broker, port, topic) -> None:
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.topic = topic

    def connect_mqtt(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(
            mqtt_client.CallbackAPIVersion.VERSION1, self.client_id
        )
        client.connect(self.broker, self.port)
        # client.username_pw_set(username, password)
        client.on_connect = on_connect
        return client

    def publish(self, client, msg):
        result = client.publish(self.topic, msg)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{self.topic}`")
        else:
            print(f"Failed to send message to topic {self.topic}")
        return

    def subscribe(self, client: mqtt_client):
        def on_message(client, userdata, msg):
            print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")

        client.subscribe(self.topic)
        client.on_message = on_message
