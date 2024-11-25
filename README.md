# WL-Local-Server

## Introduction

This repo provides code to receive and synchronize IMU, GPS, and WiFi data using an MQTT server. The synchronized data is then sent to a MinIO database, interacting with Elasticsearch to stream data on a local server, demonstrating how to interact with Elasticsearch to store, retrieve, and manage data effectively.

## Table of Contents:

1. [Problem Definition](#problem-definition)
2. [Demographics](#demographics)
3. [Goals and Challenges](#goals-and-challenges)
4. [Usage](#usage)
5. [Application Information](#application-information)
6. [Set Up](#set-up)
7. [Project Roadmap](#project-roadmap)

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

This project directory consists of four nodes:

- [minio_script](./src/minio_script.py): This file allows the user to parse the received queries from MQTT and retrieve the data from MinIO based on the receving queries. In addition, the file can list all objects in the specified bucket to verify if the data has been stored or not and showing the available dataset in the local MinIO database by using User ID and Timestamp.

- [data_processing](./src/data_processing.py): This file will be the center of data manipulation and data filtering purposes. Later on, the project will be moving onto building AI models that requires filtered and modified datas. This file has implemented the following features:
        - Filter out a set of data from MinIO based on the given hour range, minute range, and what type of column (the key of the dataframe) does the user wants.
        - Add a new column of data into a list of given dataframes.
        - Delete a column of data in the list of given dataframes.
        - Updates the latest modified data from the local machine to the MinIO database.

- [retrieve_data_from_minio](./src/retrieve_data_from_minio.py): This file acts as a bridge between MQTT and MinIO. In this file, we implemented the function that parses information from the MQTT subscriber and use the User ID and timestamp from the parsed information to receive parquet files from MinIO. In addition, we implement functions that get live-time User ID and timestamp whenever the application is running and we send the modified and parsed data back to MQTT to update the latest information to the map on the application. 

- [sync_minio_and_mqtt](./src/sync_minio_and_mqtt.py): This file acts as a bridge between MQTT and MinIO. In this file, we implemented the function that parses information from the MQTT subscriber and use the User ID and timestamp from the parsed information to receive parquet files from MinIO. In addition, we implement functions that get live-time User ID and timestamp whenever the application is running and we send the modified and parsed data back to MQTT to update the latest information to the map on the application. 
  
The script connects to an existing MinIO server located in the Wiloc SSH, sending accelerometer, gyroscope, GPS, and WiFi data. This data is buffered and synchronized based on timestamps in the imu_gps_publisher.py script. The node also handles errors, and reconnection attempts, and shuts down gracefully when interrupted (Ctrl+C).

- [imu_gps_publisher.py](./src/imu_gps_publisher.py): An MQTT script that connects to the MQTT server that is receiving WiFi data from 4 RPI's as well as receiving GPS and IMU data from an Android Phone running the WLMap application. It establishes an MQTT connection to the Wiloc SSH server (tcp://128.205.218.189:1883) and listens for incoming GPS, IMU, and WiFi data, extracting accelerometer and gyroscope XYZ values, latitude and longitude values, timestamps, and WiFI routing information. These values are then packaged into a MinIO bucket and published to the Wiloc MinIO server. The script also handles connection events, errors, and graceful shutdowns when receiving a termination signal (Ctrl+C).

- [MQTT_Handler.py](./src/MQTT_Handler.py): A basic MQTT Handler class that can subscribe and publish to a server topic. This MQTT Handler class was made as a reference for how an MQTT Handler should be formatted, being used and modified in the imu_gps_publisher.py to receive IMU and GPS data, which is then published to a MinIO server being run on the Wiloc server. The format of this MQTT Handler can be used for future MQTT connections.

- [Subscriber.py](./subscriber.py): A subscriber that creates and MQTT connection with a rasberry pi in order to send wifi data to the Wiloc SSH server (tcp://128.205.218.189:1883) This data is then synchronized and sent to MinIO to be retrieved upon user request. In order to get more information on retrieving and using the wifi data with the RPI please see https://github.com/ucsdwcsng/wiros_csi_node for more information about how to start the wiros node on the Rasberry Pi. 


### Application Information:

In order to collect the accelerometer, gyroscope, GPS, and WiFI readings, we use an application called [MQTT](https://github.com/eclipse/mosquitto).

- There are various ways to retrieve the Inertial Measurement(IMU) and GPS readings and from the phone. However, the imu_gps_publisher script achieves this by connecting the Android phone to the Wiloc MQTT server and publishing new IMU and GPS data every 500ms.

### Set Up:

### 1: Install MQTT and MinIO Python packages.

- To install the IMU and GPS synchronization package, clone this repository into a directory of your choosing and install the following Python packages from your command line:

        pip3 install paho-mqtt
        pip3 install minio

### 2: Run the WLMap Application.

- Next travel to Android Studio and run the WLMap application. If you have not already cloned the WLMap repository, follow this [link](https://github.com/WS-UB/WLMap).

- If you are prompted to create an emulation Android device, select a "Google Pixel 7a."

- Once running the WLMap application, select the "Data Collection" option on the main menu.

- Open LogCat and confirm that an MQTT connection has been established and that IMU and GPS data is being streamed to the WILOC server. If you are receiving a connection error in the LogCat terminal, restart the application.

### 3: Enable the MinIO server.

- ssh into the WILOC server:

```
        ssh wiloc@128.205.218.189
        wiloc@128.205.218.189's password: robot_wireless
```

- When in the WILOC server, run the following command:

```
        MINIO_ROOT_USER=admin MINIO_ROOT_PASSWORD=password ./minio server /mnt/data --console-address ":9001"
```

### 4: Run the IMU/GPS/WiFi synchronizer

- If the WiFi router and Raspberry Pi are connected and sending WiFi data, run the imu_gps_WiFi_publisher.py script.

- If the WiFi router and Raspberry Pi are NOT connected, run the sync_WiFi_test.py script, and in a separate terminal, run the following commands.

```
        cd ~Pathname to your WL-Local-Server repo~
        python3 WiFi_test.py
```

- To install the Elastic search package, clone this repository into a directory of your choosing and install the following Debian packages from your command line:

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

## Project Roadmap

### Server-side Data Management

Elasticsearch Application: https://github.com/elastic/elasticsearch

- [x] IMU data is received from the Android Phone via an MQTT Handler.
- [x] GPS data is received from the Android Phone via an MQTT Handler.
- [x] IMU and GPS data are synchronized within 500ms of each other.
- [x] Synchronized data can be sent to a MinIO server.
- [x] WiFi data is received from the RPI via an MQTT Handler.
- [x] WiFi data is synchronized with IMU and GPS data within 500ms of each other.
