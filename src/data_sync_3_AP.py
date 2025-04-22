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
WIFI_CS1 = "/csi-ap1"
WIFI_CS2 = "/csi-ap2"
WIFI_CS3 = "/csi-ap3"
LIST_OF_TOPICS = [TOPIC, IMU_DATA, GPS_DATA, WIFI_CS1, WIFI_CS2, WIFI_CS3]
CLIENT_ID = "".join(random.choices((string.ascii_letters + string.digits), k=6))
MARGIN = 500  # In ms


class IMU_GPS_publisher:
    def __init__(
        self,
        client_id: str,
        broker: str,
        port: int,
        topics: list[str],
        num_of_routers: int,
    ):
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.topics = topics
        self.accelerator_list = []
        self.gyroscope_list = []
        self.GPS_list = []
        self.GPS_RAW_list = []
        self.WiFi_CSI_1 = []
        self.WiFi_CSI_2 = []
        self.WiFi_CSI_3 = []
        self.CLIENT = None
        self.num_of_routers = num_of_routers
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
                if data[0] == "GPS_RAW":  # Handles GPS data that has been recieved
                    # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
                    tag = data[0]
                    device_id = data[1]
                    time_stamp = data[2]
                    lat = data[3]
                    long = data[4]
                    self.GPS_RAW_list = [tag, device_id, time_stamp, lat, long]
            if msg.topic == "/csi-ap1":
                raw_data = msg.payload.decode()
                wifiData = get_WiFi_data(raw_data=raw_data)
                self.WiFi_CSI_1 = wifiData
            if msg.topic == "/csi-ap2":
                raw_data = msg.payload.decode()
                wifiData = get_WiFi_data(raw_data=raw_data)
                self.WiFi_CSI_2 = wifiData
            if msg.topic == "/csi-ap3":
                raw_data = msg.payload.decode()
                wifiData = get_WiFi_data(raw_data=raw_data)
                self.WiFi_CSI_3 = wifiData

            if self.num_of_routers == 1:
                timestamp_avg = one_router(
                    self.accelerator_list,
                    self.gyroscope_list,
                    self.GPS_list,
                    self.GPS_RAW_list,
                    [self.WiFi_CSI_1, self.WiFi_CSI_2, self.WiFi_CSI_3],
                )
                if (timestamp_avg <= MARGIN) and (timestamp_avg != 0):
                    self.publish()

            if self.num_of_routers == 2:
                timestamp_avg = two_routers(
                    self.accelerator_list,
                    self.gyroscope_list,
                    self.GPS_list,
                    self.GPS_RAW_list,
                    [self.WiFi_CSI_1, self.WiFi_CSI_2, self.WiFi_CSI_3],
                )
                if (timestamp_avg <= MARGIN) and (timestamp_avg != 0):
                    self.publish()

            if self.num_of_routers == 3:
                timestamp_avg = three_routers(
                    self.accelerator_list,
                    self.gyroscope_list,
                    self.GPS_list,
                    self.GPS_RAW_list,
                    [self.WiFi_CSI_1, self.WiFi_CSI_2, self.WiFi_CSI_3],
                )
                if (timestamp_avg <= MARGIN) and (timestamp_avg != 0):
                    self.publish()

        for topic in self.topics:
            client.subscribe(topic)
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

        GPS_RAW_lat = float(self.GPS_RAW_list[3])
        GPS_RAW_long = float(self.GPS_RAW_list[4])

        device_id = self.GPS_list[1]

        test_list = [
            self.GPS_list,
            self.GPS_RAW_list,
            self.accelerator_list,
            self.gyroscope_list,
            [self.WiFi_CSI_1, self.WiFi_CSI_2, self.WiFi_CSI_3],
        ]
        print(f"Send `{test_list}` to MinIO bucket")

        self.accelerator_list = []
        self.gyroscope_list = []
        self.GPS_list = []
        self.GPS_RAW_list = []

        user_data = json.dumps(
            [
                {
                    "user_id": device_id,
                    "timestamp": data_timestamp,
                    "IMU": {"gyro": gyro_xyz, "accel": accel_xyz},
                    "GPS": {"latitude": GPS_lat, "longitude": GPS_long},
                    "GPS_RAW": {"latitude": GPS_RAW_lat, "longitude": GPS_RAW_long},
                    "WiFi": {
                        "WiFi-AP-1": self.WiFi_CSI_1,
                        "WiFi-AP-2": self.WiFi_CSI_2,
                        "WiFi-AP-3": self.WiFi_CSI_3,
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


def get_WiFi_data(raw_data: str) -> list[str]:
    formatted_data = raw_data.replace("\n", ",")
    list_of_data = extract_lists(formatted_data)
    csi_real = f"csi_r: {list_of_data[1]}"
    csi_imag = f"csi_i: {list_of_data[2]}"
    AP_Loc = f"AP Location: {list_of_data[3]}"
    AP_L1 = f"AP L1: {list_of_data[4]}"
    AP_L2 = f"AP L2: {list_of_data[5]}"
    data = remove_lists(formatted_data)
    wifi_timestamp = data[14].split(";")[1]
    rssi = data[1]
    ap_id = data[11]
    chan = data[2]
    bw = data[3]
    mcs = data[12]
    nsub = data[7]
    nrows = data[9]
    ncols = data[10]
    rx_id = data[13]
    wiFiData = [
        wifi_timestamp,
        csi_imag,
        csi_real,
        rssi,
        ap_id,
        chan,
        bw,
        mcs,
        nsub,
        nrows,
        ncols,
        rx_id,
        AP_Loc,
        AP_L1,
        AP_L2,
    ]
    return wiFiData


def one_router(accelerator_list, gyroscope_list, GPS_list, GPS_RAW_list, WiFi_lists):
    WiFi_CSI1 = WiFi_lists[0]
    WiFi_CSI2 = WiFi_lists[1]
    WiFi_CSI3 = WiFi_lists[2]
    if (
        len(accelerator_list) > 0
        and len(gyroscope_list) > 0  # Handles synchronization of the IMU and GPS data
        and len(GPS_list) > 0
        and len(WiFi_CSI1) > 0
    ):
        accel_id = accelerator_list[5]
        gyro_id = gyroscope_list[5]
        GPS_id = GPS_list[1]
        GPS_RAW_id = GPS_RAW_list[1]
        if accel_id == gyro_id == GPS_id == GPS_RAW_id:
            gyro_timestamp = int(gyroscope_list[4].split(".")[1])
            accel_timestamp = int(accelerator_list[4].split(".")[1])
            GPS_timestamp = int(GPS_list[2].split(".")[1])
            GPS_RAW_timestamp = int(GPS_list[2].split(".")[1])
            try:
                WiFI_timestamp = int(WiFi_CSI1[0].split(".")[1][:-3])
            except:
                WiFI_timestamp = 0
            timestamp_avg = abs(
                (
                    accel_timestamp
                    + gyro_timestamp
                    + GPS_timestamp
                    + GPS_RAW_timestamp
                    + WiFI_timestamp
                )
                / 3
            )
            return timestamp_avg
    if (
        len(accelerator_list) > 0
        and len(gyroscope_list) > 0  # Handles synchronization of the IMU and GPS data
        and len(GPS_list) > 0
        and len(WiFi_CSI2) > 0
    ):
        accel_id = accelerator_list[5]
        gyro_id = gyroscope_list[5]
        GPS_id = GPS_list[1]
        GPS_RAW_id = GPS_RAW_list[1]
        if accel_id == gyro_id == GPS_id == GPS_RAW_id:
            gyro_timestamp = int(gyroscope_list[4].split(".")[1])
            accel_timestamp = int(accelerator_list[4].split(".")[1])
            GPS_timestamp = int(GPS_list[2].split(".")[1])
            GPS_RAW_timestamp = int(GPS_list[2].split(".")[1])
            try:
                WiFI_timestamp = int(WiFi_CSI2[0].split(".")[1][:-3])
            except:
                WiFI_timestamp = 0
            timestamp_avg = abs(
                (
                    accel_timestamp
                    + gyro_timestamp
                    + GPS_timestamp
                    + GPS_RAW_timestamp
                    + WiFI_timestamp
                )
                / 3
            )
            return timestamp_avg
    if (
        len(accelerator_list) > 0
        and len(gyroscope_list) > 0  # Handles synchronization of the IMU and GPS data
        and len(GPS_list) > 0
        and len(WiFi_CSI3) > 0
    ):
        accel_id = accelerator_list[5]
        gyro_id = gyroscope_list[5]
        GPS_id = GPS_list[1]
        GPS_RAW_id = GPS_RAW_list[1]
        if accel_id == gyro_id == GPS_id == GPS_RAW_id:
            gyro_timestamp = int(gyroscope_list[4].split(".")[1])
            accel_timestamp = int(accelerator_list[4].split(".")[1])
            GPS_timestamp = int(GPS_list[2].split(".")[1])
            GPS_RAW_timestamp = int(GPS_list[2].split(".")[1])
            try:
                WiFI_timestamp = int(WiFi_CSI3[0].split(".")[1][:-3])
            except:
                WiFI_timestamp = 0
            timestamp_avg = abs(
                (
                    accel_timestamp
                    + gyro_timestamp
                    + GPS_timestamp
                    + GPS_RAW_timestamp
                    + WiFI_timestamp
                )
                / 3
            )
            return timestamp_avg
    return 0


def two_routers(accelerator_list, gyroscope_list, GPS_list, GPS_RAW_list, WiFi_lists):
    WiFi_CSI1 = WiFi_lists[0]
    WiFi_CSI2 = WiFi_lists[1]
    WiFi_CSI3 = WiFi_lists[2]
    if (
        len(accelerator_list) > 0
        and len(gyroscope_list) > 0  # Handles synchronization of the IMU and GPS data
        and len(GPS_list) > 0
        and len(WiFi_CSI1) > 0
        and len(WiFi_CSI2) > 0
    ):
        accel_id = accelerator_list[5]
        gyro_id = gyroscope_list[5]
        GPS_id = GPS_list[1]
        GPS_RAW_id = GPS_RAW_list[1]
        if accel_id == gyro_id == GPS_id == GPS_RAW_id:
            gyro_timestamp = int(gyroscope_list[4].split(".")[1])
            accel_timestamp = int(accelerator_list[4].split(".")[1])
            GPS_timestamp = int(GPS_list[2].split(".")[1])
            GPS_RAW_timestamp = int(GPS_list[2].split(".")[1])
            try:
                WiFI_CSI1_timestamp = int(WiFi_CSI1[0].split(".")[1][:-3])
                WiFI_CSI2_timestamp = int(WiFi_CSI2[0].split(".")[1][:-3])
            except:
                WiFI_CSI1_timestamp = 0
                WiFI_CSI2_timestamp = 0
            timestamp_avg = abs(
                (
                    accel_timestamp
                    + gyro_timestamp
                    + GPS_timestamp
                    + GPS_RAW_timestamp
                    + WiFI_CSI1_timestamp
                    + WiFI_CSI2_timestamp
                )
                / 5
            )
            return timestamp_avg
    if (
        len(accelerator_list) > 0
        and len(gyroscope_list) > 0  # Handles synchronization of the IMU and GPS data
        and len(GPS_list) > 0
        and len(WiFi_CSI2) > 0
        and len(WiFi_CSI3) > 0
    ):
        accel_id = accelerator_list[5]
        gyro_id = gyroscope_list[5]
        GPS_id = GPS_list[1]
        GPS_RAW_id = GPS_RAW_list[1]
        if accel_id == gyro_id == GPS_id == GPS_RAW_id:
            gyro_timestamp = int(gyroscope_list[4].split(".")[1])
            accel_timestamp = int(accelerator_list[4].split(".")[1])
            GPS_timestamp = int(GPS_list[2].split(".")[1])
            GPS_RAW_timestamp = int(GPS_list[2].split(".")[1])
            try:
                WiFI_CSI2_timestamp = int(WiFi_CSI2[0].split(".")[1][:-3])
                WiFI_CSI3_timestamp = int(WiFi_CSI3[0].split(".")[1][:-3])
            except:
                WiFI_CSI2_timestamp = 0
                WiFI_CSI3_timestamp = 0
            timestamp_avg = abs(
                (
                    accel_timestamp
                    + gyro_timestamp
                    + GPS_timestamp
                    + GPS_RAW_timestamp
                    + WiFI_CSI2_timestamp
                    + WiFI_CSI3_timestamp
                )
                / 5
            )
            return timestamp_avg

    if (
        len(accelerator_list) > 0
        and len(gyroscope_list) > 0  # Handles synchronization of the IMU and GPS data
        and len(GPS_list) > 0
        and len(WiFi_CSI1) > 0
        and len(WiFi_CSI3) > 0
    ):
        accel_id = accelerator_list[5]
        gyro_id = gyroscope_list[5]
        GPS_id = GPS_list[1]
        GPS_RAW_id = GPS_RAW_list[1]
        if accel_id == gyro_id == GPS_id == GPS_RAW_id:
            gyro_timestamp = int(gyroscope_list[4].split(".")[1])
            accel_timestamp = int(accelerator_list[4].split(".")[1])
            GPS_timestamp = int(GPS_list[2].split(".")[1])
            GPS_RAW_timestamp = int(GPS_list[2].split(".")[1])
            try:
                WiFI_CSI1_timestamp = int(WiFi_CSI1[0].split(".")[1][:-3])
                WiFI_CSI3_timestamp = int(WiFi_CSI3[0].split(".")[1][:-3])
            except:
                WiFI_CSI1_timestamp = 0
                WiFI_CSI3_timestamp = 0
            timestamp_avg = abs(
                (
                    accel_timestamp
                    + gyro_timestamp
                    + GPS_timestamp
                    + GPS_RAW_timestamp
                    + WiFI_CSI1_timestamp
                    + WiFI_CSI3_timestamp
                )
                / 5
            )
            return timestamp_avg
    return 0


def three_routers(accelerator_list, gyroscope_list, GPS_list, GPS_RAW_list, WiFi_lists):
    WiFi_CSI1 = WiFi_lists[0]
    WiFi_CSI2 = WiFi_lists[1]
    WiFi_CSI3 = WiFi_lists[2]
    if (
        len(accelerator_list) > 0
        and len(gyroscope_list) > 0  # Handles synchronization of the IMU and GPS data
        and len(GPS_list) > 0
        and len(WiFi_CSI1) > 0
        and len(WiFi_CSI2) > 0
        and len(WiFi_CSI3) > 0
    ):
        accel_id = accelerator_list[5]
        gyro_id = gyroscope_list[5]
        GPS_id = GPS_list[1]
        GPS_RAW_id = GPS_RAW_list[1]
        if accel_id == gyro_id == GPS_id == GPS_RAW_id:
            gyro_timestamp = int(gyroscope_list[4].split(".")[1])
            accel_timestamp = int(accelerator_list[4].split(".")[1])
            GPS_timestamp = int(GPS_list[2].split(".")[1])
            GPS_RAW_timestamp = int(GPS_list[2].split(".")[1])
            try:
                WiFI_CSI1_timestamp = int(WiFi_CSI1[0].split(".")[1][:-3])
                WiFI_CSI2_timestamp = int(WiFi_CSI2[0].split(".")[1][:-3])
                WiFI_CSI3_timestamp = int(WiFi_CSI3[0].split(".")[1][:-3])
            except:
                WiFI_CSI1_timestamp = 0
                WiFI_CSI2_timestamp = 0
                WiFI_CSI3_timestamp = 0

            timestamp_avg = abs(
                (
                    accel_timestamp
                    + gyro_timestamp
                    + GPS_timestamp
                    + GPS_RAW_timestamp
                    + WiFI_CSI1_timestamp
                    + WiFI_CSI2_timestamp
                    + WiFI_CSI3_timestamp
                )
                / 6
            )
            return timestamp_avg
    return 0


def run():
    print(f"Unique client ID: {CLIENT_ID}")
    NUM_OF_ROUTERS = int(input("Number of routers streaming data?: "))
    mqttHandler = IMU_GPS_publisher(
        CLIENT_ID, BROKER, PORT, LIST_OF_TOPICS, NUM_OF_ROUTERS
    )
    client = mqttHandler.connect_mqtt()
    mqttHandler.subscribe(client=client)
    client.loop_forever()


if __name__ == "__main__":
    run()
