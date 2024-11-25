import pandas as pd


def run():
    df_parquet = pd.read_parquet("output.parquet")
    df_parquet.to_csv("output.csv")


# Convert the matched strings to lists of integers


if __name__ == "__main__":
    # rospy.init_node("gps_publisher", anonymous=True)
    # target_socket = rospy.get_param("/gps_publisher/target_socket")
    # gps_pub = GPSPublisher(target_socket)
    # signal.signal(signal.SIGINT, signal_handler)
    # gps_pub.connect()
    # rospy.spin()

    run()
