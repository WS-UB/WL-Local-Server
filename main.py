from src.MQTT_Handler import MQTTHandler
from src.gps_publisher import GPSPublisher
from src.imu_publisher import IMU_publisher
import threading
from paho.mqtt import client as mqtt_client

BROKER = "128.205.218.189"
PORT = 1883
TOPIC = "test/topic"
CLIENT_ID_IMU = "receive-data-imu"
CLIENT_ID_GPS = "receive-data-gps"


def run():
    imuHandler = GPSPublisher(CLIENT_ID_GPS, BROKER, PORT, TOPIC)
    client_imu = imuHandler.connect_mqtt()
    imuHandler.subscribe(client=client_imu)

    client_imu.loop_forever()


if __name__ == "__main__":
    # rospy.init_node("gps_publisher", anonymous=True)
    # target_socket = rospy.get_param("/gps_publisher/target_socket")
    # gps_pub = GPSPublisher(target_socket)
    # signal.signal(signal.SIGINT, signal_handler)
    # gps_pub.connect()
    # rospy.spin()

    run()
