import rospy
import subprocess
import zmq
import time
from io import BytesIO
from rf_msgs.msg import Wifi

# Create ZMQ publisher
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)

#Connect to the ZMQ subscriber (local server IP)
#you will need to change this
sock.connect("tcp://128.205.218.189:5678")

#this stores the computer's hostname, used for the server to figure out which client is sending to it
host = b''


def csi_callback(msg):
    #read input ROS message raw data into buffer
    bstr = b''
    buf = BytesIO()
    msg.serialize(buf)

    #convert iobuf into byte string
    bstr = buf.getvalue()
    blen = len(bstr).to_bytes(4,'big')
    
    #send header and bytestring to ZMQ subscriber
    bmsg = '0'.encode() + b'CSI' + host + bstr
    sock.send(bmsg)
    print(f"send {time.time()}")
if __name__ == "__main__":
    #tells ROS that we are creating a node
    rospy.init_node("csi_pub") 

    #Figure out the hostname of this computer, used so the server will know where the data came from
    host = subprocess.Popen("hostname", shell=True, stdout=subprocess.PIPE).stdout.read().replace(b'\n',b'_')
    if len(host) > 20:
        host = host[:20]
    else:
        host = host.ljust(20,b'_')

    '''
    Creates a subscriber:
    - subscribe to the topic /csi_server/csi
    - receive messages of type rf_msgs.Wifi
    - Call csi_callback when you get a message
    '''
    rospy.Subscriber("/csi", Wifi, csi_callback)

    #wait forever
    rospy.spin()
