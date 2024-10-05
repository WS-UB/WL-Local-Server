#!/usr/bin/env python3

from paho.mqtt import client as mqtt_client

# import rospy
# import urllib.parse
# import websocket
# import json
# from collections import deque
# import pandas as pd
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Vector3, Quaternion
# import time
# import threading
# import signal
# import sys

BROKER = "128.205.218.189"
PORT = 1883
TOPIC = "test/topic"
CLIENT_ID = "retrieve-imu-data"


class IMU_GPS_publisher:
    def __init__(self, client_id, broker, port, topic):
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.topic = topic
        self.accelerator_list = []
        self.gyroscope_list = []
        self.GPS_list = []
        self.CLIENT = None

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
            global GPS_LIST
            data = msg.payload.decode().split(",")
            if (
                data[0] == "accelerator"
            ):  # Change to another identifier at a later point
                # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
                tag = "acelerometer_data"
                x = data[1]
                y = data[2]
                z = data[3]
                time_stamp = data[4]
                self.accelerator_list = [tag, x, y, z, time_stamp]
            if data[0] == "gyroscope":  # Change to another identifier at a later point
                # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
                tag = "gyroscope_data"
                x = data[1]
                y = data[2]
                z = data[3]
                time_stamp = data[4]
                self.gyroscope_list = [tag, x, y, z, time_stamp]
            if data[0] == "GPS":  # Change to another identifier at a later point
                # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
                tag = data[0]
                lat = data[1]
                long = data[2]
                time_stamp = data[3]
                self.GPS_list = [tag, lat, long, time_stamp]
            if (
                len(self.accelerator_list) > 0
                and len(self.gyroscope_list) > 0
                and len(self.GPS_list) > 0
            ):
                gyro_and_accel = [
                    self.GPS_list,
                    self.accelerator_list,
                    self.gyroscope_list,
                ]
                gyro_timestamp = int(self.gyroscope_list[4].split(".")[1])
                accel_timestamp = int(self.accelerator_list[4].split(".")[1])
                GPS_timestamp = int(self.GPS_list[3].split(".")[1])
                timestamp_avg = abs(
                    (accel_timestamp + gyro_timestamp + GPS_timestamp) / 3
                )
                self.accelerator_list = []
                self.gyroscope_list = []
                self.GPS_list = []
                if timestamp_avg <= 100:
                    self.publish(client=self.CLIENT, msg=str(gyro_and_accel))

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
    mqttHandler = IMU_GPS_publisher(CLIENT_ID, BROKER, PORT, TOPIC)
    client = mqttHandler.connect_mqtt()
    mqttHandler.subscribe(client=client)
    client.loop_forever()


if __name__ == "__main__":
    run()
