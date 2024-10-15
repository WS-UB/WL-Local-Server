# WiSense IMU/GPS/WiFi Publisher

## Introduction

This repo provides code to stream Elasticsearch on a local server and demonstrates how to interact with Elasticsearch to store, retrieve, and manage data effectively.

## Table of Contents:

1. [Problem Definition](#problem-definition)
2. [Demographics](#demographics)
3. [Usage](#usage)
4. [Application Information](#application-information)
5. [Set Up](#set-up)

## Problem Definition

We aim to build the back end of the Android mobile application by collecting the data from the app and passing it through a machine-learning model to improve the precision of the user's location on the map. On this current sprint, we will focus on passing the following data to the MQTT server:

- User ID
- Timestamp
- Gyroscope readings (x,y,z-values)
- Accelerometer readings (x,y,z-values)
- Latitude and longitude
- MAC address of the transmitter
- Channel number
- Number of subcarriers
- Number of rows and columns of the CSI matrix
- Bandwidths
- Signal strength
- Frame control byte
- Frame sequence number/packet number
- Real part of Channel state information (CSI)
- Imaginary part of CSI
- IP of the asus that received the packet

This repo provides code to stream and synchronize IMU, GPS, and WiFi from the MQTT Wiloc server. It can be used in combination with our MinIO object storage server implementation to create a testbed for building-scale WiFi sensing applications.
From here, the data will be queried and passed onto Elasticsearch, which is where we can apply the machine learning model. Once the data has been passed through Elasticsearch, we will store it in MinIO. Our goal is to make sure that real-life data can be passed through MQTT, Elasticsearch, and MinIO back and forth so that the user can receive the most accurate dataset in the application and improve the user's overall experience of using the map.

## Table of Contents:

1. [Problem Definition](#problem-definition)
2. [Demographics](#demographics)
3. [Goals and Challenges](#goals-and-challenges)
4. [Usage](#usage)
5. [Application Information](#application-information)
6. [Set Up](#set-up)
7. [Project Roadmap](#project-roadmap)

## Problem Definition

We aim to create a stable Android mobile application for indoor navigation and WiFi-based data collection. This semester, we will focus on resolving technical issues with the data collection app, enhancing the user interface, and integrating server-side data processing using AWS. Additionally, we will develop a functional navigational interface similar to Google Maps, enabling users to track their indoor location within large buildings like malls and airports. In the long term, we aspire to deploy a fully functional, scalable system that enables seamless indoor navigation by utilizing WiFi signals and real-time data collection. By leveraging machine learning models, we will enhance accuracy in indoor positioning, ensuring privacy and efficiency through the use of hashed user data. Our goal is to provide a robust and open-source platform that can be adapted for various large-scale indoor environments.

## Demographics

This application will be designed and used by University at Buffalo students and faculty, with the goal being that the application is made into an open-source platform that can be adapted for various large-scale indoor environments.

## Goals and Challenges

Our current goals for the server-side aspect of this project include:

- Resolving technical issues with the data collection app.
- Storing user/device data using server integration.
- Integrating server-side data processing using AWS.
- Retrieving server-side data to update user position.
- Gather and store user navigation data in a database.
- Use stored reliable data for an A.I. training model.

### Usage:

This project directory consists of two nodes:

- [elastic_read](./src/elastic_read.py): This file allows the user to access the local MinIO database and insert the JSON data from MinIO to the local Elasticsearch database. In addition, this file can interact with the local Elasticsearch database by inserting new data queries from MinIO, removing available datasets from Elasticsearch, and showing available datasets in the local Elasticsearch database.
- [minio_script](./src/minio_script.py): This file allows the user to parse the received queries from Elasticsearch and store them in MinIO with User ID and Timestamp. In addition, the file can list all objects in the specified bucket to verify if the data has been stored or not and showing the available dataset in the local MinIO database by using User ID and Timestamp.

### Application Information:

In order to pass the dataset through the machine learning model, we use an application called Elasticsearch, where all the data can be cleaned, visualized, and modified.

This project directory consists of two Python scripts:

- [imu_gps_publisher.py](./src/imu_gps_publisher.py): An MQTT script that connects to the MQTT server that is receiving WiFi data from 4 RPI's as well as receiving GPS and IMU data from an Android Phone running the WLMap application. It establishes an MQTT connection to the Wiloc SSH server (tcp://128.205.218.189:1883) and listens for incoming GPS, IMU, and WiFi data, extracting accelerometer and gyroscope XYZ values, latitude and longitude values, timestamps, and WiFI routing information. These values are then packaged into a MinIO bucket and published to the Wiloc MinIO server. The script also handles connection events, errors, and graceful shutdowns when receiving a termination signal (Ctrl+C).

- [MQTT_Handler.py](./src/MQTT_Handler.py): A basic MQTT Handler class that can subscribe and publish to a server topic. This MQTT Handler class was made as a reference for how an MQTT Handler should be formatted, being used and modified in the imu_gps_publisher.py to receive IMU and GPS data, which is then published to a MinIO server being run on the Wiloc server. The format of this MQTT Handler can be used for future MQTT connections.

- [minio_script](./src/minio_script.py): A MinIO script designed to connect an existing MinIO server to send and receive data to and from the database. The script connects to an existing MinIO server located in the Wiloc SSH, sending accelerometer, gyroscope, GPS, and WiFi data. This data is buffered and synchronized based on timestamps in the imu_gps_publisher.py script. The node also handles errors, and reconnection attempts, and shuts down gracefully when interrupted (Ctrl+C).

### Application Information:

In order to collect the accelerometer, gyroscope, GPS, and WiFI readings, we use an application called [MQTT](https://github.com/eclipse/mosquitto).

- There are various ways to retrieve the Inertial Measurement(IMU) and GPS readings and from the phone. However, the imu_gps_publisher script achieves this by connecting the Android phone to the Wiloc MQTT server and publishing new IMU and GPS data every 500ms.

This project directory consists of two Python scripts:

- [imu_gps_publisher.py](./src/imu_gps_publisher.py): An MQTT script that connects to the MQTT server that is receiving WiFi data from 4 RPI's as well as receiving GPS and IMU data from an Android Phone running the WLMap application. It establishes an MQTT connection to the Wiloc SSH server (tcp://128.205.218.189:1883) and listens for incoming GPS, IMU, and WiFi data, extracting accelerometer and gyroscope XYZ values, latitude and longitude values, timestamps, and WiFI routing information. These values are then packaged into a MinIO bucket and published to the Wiloc MinIO server. The script also handles connection events, errors, and graceful shutdowns when receiving a termination signal (Ctrl+C).

- [MQTT_Handler.py](./src/MQTT_Handler.py): A basic MQTT Handler class that can subscribe and publish to a server topic. This MQTT Handler class was made as a reference for how an MQTT Handler should be formatted, being used and modified in the imu_gps_publisher.py to receive IMU and GPS data, which is then published to a MinIO server being run on the Wiloc server. The format of this MQTT Handler can be used for future MQTT connections.

- [minio_script](./src/minio_script.py): A MinIO script designed to connect an existing MinIO server to send and receive data to and from the database. The script connects to an existing MinIO server located in the Wiloc SSH, sending accelerometer, gyroscope, GPS, and WiFi data. This data is buffered and synchronized based on timestamps in the imu_gps_publisher.py script. The node also handles errors, and reconnection attempts, and shuts down gracefully when interrupted (Ctrl+C).

### Application Information:

In order to collect the accelerometer, gyroscope, GPS, and WiFI readings, we use an application called [MQTT](https://github.com/eclipse/mosquitto).

- There are various ways to retrieve the Inertial Measurement(IMU) and GPS readings and from the phone. However, the imu_gps_publisher script achieves this by connecting the Android phone to the Wiloc MQTT server and publishing new IMU and GPS data every 500ms.

### Set Up:

- To install this package, clone this repository into a directory of your choosing and install the following Debian packages from your command line:

```
    wget https://artifacts.elastic.co/downloads/elasticsearch/elasticsearch-8.15.2-amd64.deb
    wget https://artifacts.elastic.co/downloads/elasticsearch/elasticsearch-8.15.2-amd64.deb.sha512
    shasum -a 512 -c elasticsearch-8.15.2-amd64.deb.sha512
    sudo dpkg -i elasticsearch-8.15.2-amd64.deb
```

- Next, you run Elasticsearch by using the following command lines:

```
    sudo /bin/systemctl daemon-reload
    sudo /bin/systemctl enable elasticsearch.service
    sudo systemctl start elasticsearch.service
```

- To install this package, clone this repository into a directory of your choosing and install the following Python packages from your command line:

        pip3 install paho-mqtt
        pip3 install minio

## Project Roadmap

### Server-side Data Management

Elasticsearch Application: https://github.com/elastic/elasticsearch

- [x] IMU data is received from the Android Phone via an MQTT Handler.
- [x] GPS data is received from the Android Phone via an MQTT Handler.
- [x] IMU and GPS data are synchronized within 500ms of each other.
- [x] Synchronized data can be sent to a MinIO server.
- [ ] WiFi data is received from the RPI via an MQTT Handler.
- [ ] WiFi data is synchronized with IMU and GPS data within 500ms of each other.
