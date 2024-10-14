import rospy
import subprocess
import zmq
import time
from io import BytesIO
from rf_msgs.msg import Wifi
from minio import Minio
from minio.error import S3Error

# Create MinIO client
minio_client = Minio(
    "100.100.100.100:9000",  # MinIO server address
    access_key="minio",  # MinIO access key
    secret_key="minio",  # MinIO secret key
    secure=False  # Set to True if using HTTPS
)

# Function to upload to MinIO
def upload_to_minio(data, object_name, bucket_name="csi-data"):
    """Uploads the data to MinIO directly from memory"""
    try:
        if not minio_client.bucket_exists(bucket_name):
            minio_client.make_bucket(bucket_name)
            print(f"Bucket '{bucket_name}' created.")
        
        # Upload data as an object to MinIO
        minio_client.put_object(
            bucket_name, object_name, BytesIO(data), len(data)
        )
        print(f"Data uploaded to MinIO: {object_name}")
    except S3Error as err:
        print(f"MinIO Error: {err}")

# Create ZMQ publisher
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.connect("tcp://100.100.100.100:1000")  # Connect to the ZMQ subscriber

# Stores the computer's hostname
host = b''

def csi_callback(msg):
    """Callback to handle incoming CSI data"""
    # Read input ROS message raw data into buffer
    buf = BytesIO()
    msg.serialize(buf)

    # Convert iobuf into byte string
    bstr = buf.getvalue()
    blen = len(bstr).to_bytes(4, 'big')

    # Send data through ZMQ
    bmsg = '0'.encode() + b'CSI' + host + bstr
    sock.send(bmsg)
    print(f"Data sent via ZMQ at {time.time()}")

    # Upload data to MinIO
    object_name = f"csi_data_{int(time.time()*1000)}.bin"
    upload_to_minio(bstr, object_name)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("csi_pub")

    # Get the hostname of the computer
    host = subprocess.Popen("hostname", shell=True, stdout=subprocess.PIPE).stdout.read().replace(b'\n', b'_')
    if len(host) > 20:
        host = host[:20]
    else:
        host = host.ljust(20, b'_')

    # Subscribe to the /csi topic
    rospy.Subscriber("/csi", Wifi, csi_callback)

    # Keep the node running
    rospy.spin()
