# WiSense IMU/GPS Publisher

## Introduction
This repo provides code to stream IMU and GPS data over a websocket from the [Sensor Server](https://github.com/umer0586/SensorServer) android app. It can be used in combination with our MinIO CSI server implementation to create a testbed for building-scale WiFi sensing applications.

## Table of Contents:
1. [Usage](#usage)
2. [Application Information](#application-information)
3. [Set Up](#set-up)
4. [Citations](#citations)

### Usage:
This project directory consists of two nodes:

-  [gps_publisher.py](./src/gps_publisher.py): A ROS node that connects to a WebSocket server to receive GPS data and publish it as a ROS topic. It establishes a WebSocket connection to a specified URL (ws://localhost:8090/gps) and listens for incoming GPS data in JSON format, extracting latitude and longitude values. These values are then packaged into a NavSatFix message and published to the /gps ROS topic. The script also handles connection events, errors, and graceful shutdown on receiving a termination signal (Ctrl+C). Additionally, it periodically sends a "getLastKnownLocation" request to the WebSocket server to fetch the latest GPS coordinates.
- [imu_publisher](./src/imu_publisher.py): A ROS node designed to collect and publish IMU (Inertial Measurement Unit) data from an Android device to a ROS topic. The script connects to a WebSocket server using a specified URL(ws://localhost:8090/sensors/connect?types={self.encoded_types}), retrieving accelerometer and gyroscope data. This data is buffered, synchronized based on timestamps, and then published as an Imu message to the /imu ROS topic. The node also handles errors, reconnection attempts, and shuts down gracefully when interrupted (Ctrl+C). The data synchronization ensures that the IMU data is aligned in time before being published.

### Application Information:
In order to collect the accelerometer, gyroscope and GPS readings, we use an application called [Sensor Server](https://github.com/umer0586/SensorServer). 

- There are various ways to retrieve the Inertial Measurement(IMU) and GPS readings and  from the phone. However, the imu_publisher package achieves it by connecting the Android phone to the device via hotspot.

### Set Up:
- To install this package, clone this repository in the catkin_ws directory and in the repository's root folder run catkin build:

        cd /home/user/catkin_ws/src
        git clone https://github.com/WS-UB/imu_publisher.git
        cd imu_publisher
        catkin build

- Source the packages and confirm the paths are included:

        source ./devel/setup.bash
        echo $ROS_PACKAGE_PATH

- Install the **SensorServer** application from [Installation](https://github.com/umer0586/SensorServer/blob/main/README.md#installation:~:text=connected%20android%20phone.-,Installation,-OR)
- Once the application is downloaded, open it and click on the menu bar in the top left corner. 
- Choose settings and enable the *Use Hotspot* Option.
- Go back to the *Server* in the bottom left corner and click on start.
- Set the `target_socket` parameter in `gps_publisher.launch` and `imu_publisher.launch` to the websocket displayed by the app.
- If the connection is established, you will see a pop up notification on the application in the middle bottom at the *Connections* icon. Since we are retrieving the IMU data and GPS readings, we should be able to see two connections.

### Citations:

SensorServer Application: https://github.com/umer0586/SensorServer/blob/main/README.md
