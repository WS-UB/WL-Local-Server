from src.MQTT_Handler import MQTTHandler
import threading
from paho.mqtt import client as mqtt_client
from datetime import datetime
import re
import pandas as pd

BROKER = "128.205.218.189"
PORT = 1883
TOPIC = "/csi"
CLIENT_ID_WIFI = "send-data-WiFi"


def run():
    wifi_Handler = MQTTHandler(CLIENT_ID_WIFI, BROKER, PORT, TOPIC)
    client_wifi = wifi_Handler.connect_mqtt()
    wifi_data = "1, [0, 26, 43, 60, 77, 94], 6, 64, 2, 4, 20, 9, -65, 8, 123, [0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8], 192.168.1.2, 456"

    while True:
        wifi_Handler.publish(client=client_wifi, msg=wifi_data)


# Convert the matched strings to lists of integers


if __name__ == "__main__":
    # rospy.init_node("gps_publisher", anonymous=True)
    # target_socket = rospy.get_param("/gps_publisher/target_socket")
    # gps_pub = GPSPublisher(target_socket)
    # signal.signal(signal.SIGINT, signal_handler)
    # gps_pub.connect()
    # rospy.spin()

    run()
