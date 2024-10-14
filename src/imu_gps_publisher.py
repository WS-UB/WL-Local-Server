from paho.mqtt import client as mqtt_client
from minio_script import store_received_data, list_objects_in_bucket
import json

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

# RUN "minio server /Users/harrisonmoore/data" in your terminal to start server

BROKER = "128.205.218.189"
PORT = 1883
TOPIC = "test/topic"
WIFI = "/csi"
CLIENT_ID = "retrieve-imu-data"
MARGIN = 500  # In ms


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
        global MARGIN

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
            if msg.topic == "test/topic":
                if (
                    data[0]
                    == "accelerator"  # Handles accelerometer data that has been recieved
                ):  # Change to another identifier at a later point
                    # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
                    tag = "acelerometer_data"
                    x = data[1]
                    y = data[2]
                    z = data[3]
                    time_stamp = data[4]
                    device_id = data[5]
                    self.accelerator_list = [tag, x, y, z, time_stamp, device_id]
                if (
                    data[0] == "gyroscope"
                ):  # Handles gyroscope data that has been recieved
                    # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
                    tag = "gyroscope_data"
                    x = data[1]
                    y = data[2]
                    z = data[3]
                    time_stamp = data[4]
                    device_id = data[5]
                    self.gyroscope_list = [tag, x, y, z, time_stamp, device_id]
                if data[0] == "GPS":  # Handles GPS data that has been recieved
                    # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
                    tag = data[0]
                    device_id = data[1]
                    time_stamp = data[2]
                    lat = data[3]
                    long = data[4]
                    self.GPS_list = [tag, device_id, time_stamp, lat, long]
            if (
                len(self.accelerator_list) > 0
                and len(self.gyroscope_list)
                > 0  # Handles synchronization of the IMU and GPS data
                and len(self.GPS_list) > 0
            ):
                if (
                    self.accelerator_list[5]
                    == self.gyroscope_list[5]
                    == self.GPS_list[1]
                ):
                    gyro_and_accel = [
                        self.GPS_list,
                        self.accelerator_list,
                        self.gyroscope_list,
                    ]
                gyro_timestamp = int(self.gyroscope_list[4].split(".")[1])
                accel_timestamp = int(self.accelerator_list[4].split(".")[1])
                GPS_timestamp = int(self.GPS_list[2].split(".")[1])
                timestamp_avg = abs(
                    (accel_timestamp + gyro_timestamp + GPS_timestamp) / 3
                )
                if timestamp_avg <= MARGIN:
                    self.publish()

        client.subscribe(self.topic)
        client.on_message = on_message

    def publish(
        self,
    ):  # Packages IMU and GPS data into JSON to sent to the MinIO bucket
        data_timestamp: str = self.GPS_list[2]
        gyro_xyz = [
            float(self.gyroscope_list[1]),
            float(self.gyroscope_list[2]),
            float(self.gyroscope_list[3]),
        ]

        accel_xyz = [
            float(self.accelerator_list[1]),
            float(self.accelerator_list[2]),
            float(self.accelerator_list[3]),
        ]

        GPS_lat = float(self.GPS_list[3])
        GPS_long = float(self.GPS_list[4])

        device_id = self.GPS_list[1]

        self.accelerator_list = []
        self.gyroscope_list = []
        self.GPS_list = []

        user_data = json.dumps(
            {
                "user_id": device_id,
                "timestamp": data_timestamp,
                "IMU": {"gyro": gyro_xyz, "accel": accel_xyz},
                "GPS": {"latitude": GPS_lat, "longitude": GPS_long},
                "WiFi": {
                    "csi_imag": 0.5,
                    "csi_real": 0.6,
                    "rssi": -70,
                    "ap_id": "AP123",
                },
                "Channel": {
                    "chan": 1,
                    "channel": 36,
                    "bw": 20,
                    "nss": 2,
                    "ntx": 1,
                    "nrx": 2,
                    "mcs": 7,
                },
            }
        )
        store_received_data(user_data)

        return


def run():
    mqttHandler = IMU_GPS_publisher(CLIENT_ID, BROKER, PORT, TOPIC)
    client = mqttHandler.connect_mqtt()
    mqttHandler.subscribe(client=client)
    client.loop_forever()


if __name__ == "__main__":
    run()
