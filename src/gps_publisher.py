#!/usr/bin/env python3
from paho.mqtt import client as mqtt_client

# import rospy
# import websocket
# import json
# from time import sleep
# import threading
# from sensor_msgs.msg import NavSatFix
# import sys
# import signal


BROKER = "128.205.218.189"
PORT = 1883
TOPIC = "test/topic"
CLIENT_ID = "retrieve-gps-data"


class GPSPublisher:
    def __init__(self, client_id, broker, port, topic):
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.topic = topic
        self.CLIENT = None
        self.GPS_LIST = []

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
        self.CLIENT = client
        return client

    def subscribe(self, client: mqtt_client):
        def on_message(client, userdata, msg):
            data = msg.payload.decode().split(",")
            if data[0] == "GPS":  # Change to another identifier at a later point
                # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
                tag = data[0]
                timeStamp = data[1]
                lat = data[2]
                long = data[3]
                self.GPS_LIST = [tag, timeStamp, lat, long]
                self.publish(client=self.CLIENT, msg=str(self.GPS_LIST))

        client.subscribe(self.topic)
        client.on_message = on_message

    def publish(self, client, msg):
        result = client.publish(self.topic, msg)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{self.topic}`")
        else:
            print(f"Failed to send message to topic {self.topic}")
        return


def run():
    mqttHandler = GPSPublisher(CLIENT_ID, BROKER, PORT, TOPIC)
    client = mqttHandler.connect_mqtt()
    mqttHandler.subscribe(client=client)
    client.loop_forever()


if __name__ == "__main__":
    # rospy.init_node("gps_publisher", anonymous=True)
    # target_socket = rospy.get_param("/gps_publisher/target_socket")
    # gps_pub = GPSPublisher(target_socket)
    # signal.signal(signal.SIGINT, signal_handler)
    # gps_pub.connect()
    # rospy.spin()

    run()
