# WiSense IMU/GPS/WiFi Publisher

## Introduction
This repo provides code to stream and synchronize IMU, GPS, and WiFi from the MQTT Wiloc server. It can be used in combination with our MinIO object storage server implementation to create a testbed for building-scale WiFi sensing applications.

## Table of Contents:
1. [Usage](#usage)
2. [Application Information](#application-information)
3. [Set Up](#set-up)

### Usage:
This project directory consists of two Python scripts:

-  [imu_gps_publisher.py](./src/imu_gps_publisher.py): An MQTT script that connects to the MQTT server that is receiving WiFi data from 4 RPI's as well as receiving GPS and IMU data from an Android Phone running the WLMap application. It establishes an MQTT connection to the Wiloc SSH server (tcp://128.205.218.189:1883) and listens for incoming GPS, IMU, and WiFi data, extracting accelerometer and gyroscope XYZ values, latitude and longitude values, timestamps, and WiFI routing information. These values are then packaged into a MinIO bucket and published to the Wiloc MinIO server. The script also handles connection events, errors, and graceful shutdowns when receiving a termination signal (Ctrl+C).

- [minio_script](./src/imu_publisher.py): A MinIO script designed to connect an existing MinIO server to send and receive data to and from the database. The script connects to an existing MinIO server located in the Wiloc SSH, sending accelerometer, gyroscope, GPS, and WiFi data. This data is buffered and synchronized based on timestamps in the imu_gps_publisher.py script. The node also handles errors, and reconnection attempts, and shuts down gracefully when interrupted (Ctrl+C). 

### Application Information:
In order to collect the accelerometer, gyroscope, GPS, and WiFI readings, we use an application called [MQTT](https://github.com/eclipse/mosquitto). 

- There are various ways to retrieve the Inertial Measurement(IMU) and GPS readings and from the phone. However, the imu_gps_publisher script achieves this by connecting the Android phone to the Wiloc MQTT server and publishing new IMU and GPS data every 500ms.

### Set Up:
- To install this package, clone this repository into a directory of your choosing and install the following Python packages from your command line:

        pip3 install paho-mqtt
        pip3 install minio



