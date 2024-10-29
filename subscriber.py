import rospy
import subprocess

import time
from io import BytesIO
from rf_msgs.msg import Wifi
import paho.mqtt.client as mqtt_client
import random

# Create MQTT publisher
address = "128.205.218.189"
port = 1883
client_id = f"python-mqtt-{random.randint(0, 1000)}"
CLIENT = mqtt_client.Client("client")
topic = "/csi"


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(address, port)
    return client


def csi_callback(msg):
    global CLIENT
    # read input ROS message raw data into buffer
    bstr = b""
    buf = BytesIO()
    msg.serialize(buf)

    # convert iobuf into byte string
    bstr = buf.getvalue()
    blen = len(bstr).to_bytes(4, "big")

    # send header and bytestring to ZMQ subscriber
    bmsg = "0".encode() + b"CSI" + host + bstr
    CLIENT.publish("/csi", bmsg)
    print(f"send {time.time()}")


# this stores the computer's hostname, used for the server to figure out which client is sending to it
host = b""


if __name__ == "__main__":
    # tells ROS that we are creating a node
    # rospy.init_node("csi_pub")

    # Figure out the hostname of this computer, used so the server will know where the data came from
    host = (
        subprocess.Popen("hostname", shell=True, stdout=subprocess.PIPE)
        .stdout.read()
        .replace(b"\n", b"_")
    )
    if len(host) > 20:
        host = host[:20]
    else:
        host = host.ljust(20, b"_")

    """
    Creates a subscriber:
    - subscribe to the topic /csi_server/csi
    - receive messages of type rf_msgs.Wifi
    - Call csi_callback when you get a message
    """

    CLIENT = connect_mqtt()
    sub = rospy.Subscriber("/csi", Wifi, csi_callback)
    pub = rospy.Publisher("/csi", Wifi, queue_size=1)

    # wait forever
    rospy.spin()
