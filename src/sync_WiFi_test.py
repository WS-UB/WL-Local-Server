from paho.mqtt import client as mqtt_client
from minio_script import store_received_data, list_objects_in_bucket
import datetime
import re
import json
import time
import random
import string


# RUN "minio server /Users/harrisonmoore/data" in your terminal to start server

BROKER = "128.205.218.189"
PORT = 1883
IMU_DATA = "/imu"
GPS_DATA = "/gps"
TOPIC = "test/topic"
WIFI = "/csi"
LIST_OF_TOPICS = [TOPIC, IMU_DATA, GPS_DATA, WIFI]
CLIENT_ID = "".join(random.choices((string.ascii_letters + string.digits), k=6))
MARGIN = 500  # In ms


class IMU_GPS_publisher:
    def __init__(self, client_id: str, broker: str, port: int, topics: list[str]):
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.topics = topics
        self.accelerator_list = []
        self.gyroscope_list = []
        self.GPS_list = []
        self.WiFi_list = []
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
            if msg.topic == "/imu":
                data = msg.payload.decode().split(",")
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
            if msg.topic == "/gps":
                data = msg.payload.decode().split(",")
                if data[0] == "GPS":  # Handles GPS data that has been recieved
                    # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
                    tag = data[0]
                    device_id = data[1]
                    time_stamp = data[2]
                    lat = data[3]
                    long = data[4]
                    self.GPS_list = [tag, device_id, time_stamp, lat, long]
            if msg.topic == "/csi":
                raw_data = msg.payload.decode()
                list_of_data = extract_lists(raw_data)
                txmac = list_of_data[0]
                csi_real = list_of_data[1]
                csi_imag = list_of_data[2]
                data = remove_lists(raw_data)
                wifi_timestamp = str(datetime.datetime.now())
                rssi = data[8]
                ap_id = data[0]
                chan = data[2]
                bw = data[6]
                mcs = data[7]
                self.WiFi_list = [
                    wifi_timestamp,
                    csi_imag,
                    csi_real,
                    rssi,
                    ap_id,
                    chan,
                    bw,
                    mcs,
                ]

            if (
                len(self.accelerator_list) > 0
                and len(self.gyroscope_list)
                > 0  # Handles synchronization of the IMU and GPS data
                and len(self.GPS_list) > 0
                and len(self.WiFi_list) > 0
            ):
                accel_id = self.accelerator_list[5]
                gyro_id = self.gyroscope_list[5]
                GPS_id = self.GPS_list[1]
                if accel_id == gyro_id == GPS_id:
                    gyro_timestamp = int(self.gyroscope_list[4].split(".")[1])
                    accel_timestamp = int(self.accelerator_list[4].split(".")[1])
                    GPS_timestamp = int(self.GPS_list[2].split(".")[1])
                    try:
                        WiFI_timestamp = int(self.WiFi_list[0].split(".")[1][:-3])
                    except:
                        WiFI_timestamp = 0
                    timestamp_avg = abs(
                        (
                            accel_timestamp
                            + gyro_timestamp
                            + GPS_timestamp
                            + WiFI_timestamp
                        )
                        / 3
                    )
                    if timestamp_avg <= MARGIN:
                        self.publish()

        for topic in self.topics:
            client.subscribe(topic)
        client.on_message = on_message

    def publish(
        self,
    ):  # Packages IMU and GPS data into JSON to sent to the MinIO bucket
        data_timestamp: str = self.WiFi_list[0]
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

        WiFi_csi_imag = self.WiFi_list[1]
        WiFi_csi_real = self.WiFi_list[2]
        WiFi_rssi = self.WiFi_list[3]
        WiFi_ap_id = self.WiFi_list[4]
        WiFi_chan = self.WiFi_list[5]
        WiFi_bw = self.WiFi_list[6]
        WiFi_mcs = self.WiFi_list[7]

        device_id = self.GPS_list[1]

        test_list = [
            self.GPS_list,
            self.accelerator_list,
            self.gyroscope_list,
            self.WiFi_list,
        ]
        print(f"Send `{test_list}` to topic `{self.topics[0]}`")

        self.accelerator_list = []
        self.gyroscope_list = []
        self.GPS_list = []

        user_data = json.dumps(
            [
                {
                    "user_id": device_id,
                    "timestamp": data_timestamp,
                    "IMU": {"gyro": gyro_xyz, "accel": accel_xyz},
                    "GPS": {"latitude": GPS_lat, "longitude": GPS_long},
                    "WiFi": {
                        "csi_imag": WiFi_csi_imag,
                        "csi_real": WiFi_csi_real,
                        "rssi": WiFi_rssi,
                        "ap_id": WiFi_ap_id,
                    },
                    "Channel": {
                        "chan": WiFi_chan,
                        "channel": 36,
                        "bw": WiFi_bw,
                        "nss": 2,
                        "ntx": 1,
                        "nrx": 2,
                        "mcs": 7,
                    },
                }
            ]
        )

        store_received_data(user_data)

        return


def extract_lists(data: str) -> list[list[float]]:
    matches = re.findall(r"\[(.*?)\]", data)
    extracted_lists = [list(map(float, match.split(","))) for match in matches]
    return extracted_lists


def remove_lists(data: str) -> list[str]:
    cleaned_string = re.sub(r"\[.*?\]", "", data)
    # Remove any extra commas or spaces
    cleaned_string = re.sub(r",\s*", ", ", cleaned_string).strip(", ")
    split_string = cleaned_string.split(",")
    return split_string


def run():
    print(f"Unique client ID: {CLIENT_ID}")
    mqttHandler = IMU_GPS_publisher(CLIENT_ID, BROKER, PORT, LIST_OF_TOPICS)
    client = mqttHandler.connect_mqtt()
    mqttHandler.subscribe(client=client)
    client.loop_forever()


if __name__ == "__main__":
    run()
