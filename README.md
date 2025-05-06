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

## Usage:

This project directory consists of the following nodes:

- [minio_script](./src/minio_script.py): This file allows the user to parse the received queries from MQTT and store them in MinIO in the format of parquet files. More specifically, the file will parse the information from MQTT, turn it into a parquet file, and use the receiving User ID and Timestamp from MQTT as the name of the file. In addition, the file can list all parquet files in the specified bucket to verify if the data has been stored or not and show the available dataset in the local MinIO database by using the User ID and Timestamp.

- [data_processing](./src/data_processing.py): This file will be the center of data manipulation and data filtering purposes. Later on, the project will be moving onto building AI models that requires filtered and modified datas. This file has implemented the following features: - Filter out a set of data from MinIO based on the given hour range, minute range, and what type of column (the key of the dataframe) does the user wants. - Add a new column of data into a list of given dataframes. - Delete a column of data in the list of given dataframes. - Updates the latest modified data from the local machine to the MinIO database.

- [retrieve_data_from_minio](./src/retrieve_data_from_minio.py): This file acts as a bridge between MQTT and MinIO. In this file, we implemented the function that parses information from the MQTT subscriber and use the User ID and timestamp from the parsed information to receive parquet files from MinIO. In addition, we implement functions that get live-time User ID and timestamp whenever the application is running and we send the modified and parsed data back to MQTT to update the latest information to the map on the application.

- [sync_minio_and_mqtt](./src/sync_minio_and_mqtt.py): This file acts as a bridge between MQTT and MinIO. In this file, we implemented the function that parses information from the MQTT subscriber and use the User ID and timestamp from the parsed information to receive parquet files from MinIO. In addition, we implement functions that get live-time User ID and timestamp whenever the application is running and we send the modified and parsed data back to MQTT to update the latest information to the map on the application.

The script connects to an existing MinIO server located in the Wiloc SSH, sending accelerometer, gyroscope, GPS, and WiFi data. This data is buffered and synchronized based on timestamps in the imu_gps_publisher.py script. The node also handles errors, and reconnection attempts, and shuts down gracefully when interrupted (Ctrl+C).

- [data_sync_3_AP.py](./src/data_sync_3_AP.py): A MQTT script that connects to the MQTT server that is receiving Wi-Fi data from up to 3 RPI's as well as receiving GPS and IMU data from an Android Phone running the WLMap application. It establishes an MQTT connection to the Wiloc SSH server (tcp://128.205.218.189:1883) and listens for incoming GPS, IMU, and WiFi data, extracting accelerometer and gyroscope XYZ values, latitude and longitude values, timestamps, and WiFI routing information. These values are then packaged into a MinIO bucket and published to the Wiloc MinIO server. The script also handles connection events, errors, and graceful shutdowns when receiving a termination signal (Ctrl+C).

- [data_sync_6_AP.py](./src/data_sync_6_AP.py): The same as the [data_sync_3_AP.py](./src/data_sync_3_AP.py) script, but it accounts for six APs.

- [MQTT_Handler.py](./src/MQTT_Handler.py): A basic MQTT Handler class that can subscribe and publish to a server topic. This MQTT Handler class was made as a reference for how an MQTT Handler should be formatted, being used and modified in the imu_gps_publisher.py to receive IMU and GPS data, which is then published to a MinIO server being run on the Wiloc server. The structure of this MQTT Handler can be used for future MQTT connections.

- [MQTT_Handler.py](./src/MQTT_Handler.py): A basic MQTT Handler class that can subscribe and publish to a server topic. This MQTT Handler class was made as a reference for how an MQTT Handler should be formatted, being used and modified in the imu_gps_publisher.py to receive IMU and GPS data, which is then published to a MinIO server being run on the Wiloc server. The format of this MQTT Handler can be used for future MQTT connections.

- [Subscriber.py](./subscriber.py): A subscriber that creates an MQTT connection with a Raspberry PI in order to send Wifi data to the Wiloc SSH server (tcp://128.205.218.189:1883) This data is then synchronized and sent to MinIO to be retrieved upon user request. To get more information on retrieving and using the wifi data with the RPI please see https://github.com/ucsdwcsng/wiros_csi_node for more information about how to start the wiros node on the Rasberry Pi.

- [key_specific_data_retrieval.py](./src/key_specific_data_retrieval.py): A script designed to automate the retrieval of datasets from a MinIO object storage server. It allows for secure connection, efficient data access, and basic preprocessing of datasets. This tool is useful for projects that involve large-scale data storage and retrieval, enabling smooth integration with machine-learning workflows or other analytical applications.


### Application Information:

- [retrieve_all.py](./src/retrieve_all.py): A python script that asks for the folder name within the MinIO bucket, printing all the contents of that folder to your output terminal.

- [csi_calibration.py](/src/data_processing/csi_calibration.py): A python script that takes the name of a MinIO folder as user input, and parses the CSI Real and CSI Imaginary data within the given folder. The CSI data is then calibrated into 234x4 matrices, applying Fast Fourier Transformations to generate 400x360 heatmap matrices. These heatmaps are then sent back into the MinIO server for use in our machine learning model.

- [constants.py](/src/data_processing/constants.py): A python script that contains subfrequency information based on the bandwidth of the WiFi CSI data being parsed. This file contains subfrequency information that is used to remove unnecessary subfreqeuncies from the CSI Imaginary and CSI Real data.

- [pipeline_utils.py](/src/data_processing/pipeline_utils.py): A python script that contains helpful tools to parse and calibrate raw CSI data.

In order to collect the accelerometer, gyroscope, GPS, and WiFI readings, we use an application called [MQTT](https://github.com/eclipse/mosquitto).

- There are various ways to retrieve the Inertial Measurement(IMU) and GPS readings and from the phone. However, the imu_gps_publisher script achieves this by connecting the Android phone to the Wiloc MQTT server and publishing new IMU and GPS data every 500ms.

## IMU/GPS/Wi-Fi Synchronization Startup:

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
        ● minio.service - MinIO
     Loaded: loaded (/etc/systemd/system/minio.service; enabled; vendor preset: enabled)
     Active: active (running) since Wed 2025-04-30 14:40:54 EDT; 3min 54s ago
       Docs: https://docs.min.io
   Main PID: 121271 (minio)
      Tasks: 37 (limit: 38162)
     Memory: 345.7M
     CGroup: /system.slice/minio.service
```

- If not, run the following to restart the MinIO server:

```
        sudo systemctl daemon-reload
        sudo systemctl start minio
        sudo systemctl enable minio
```

- Check the status of MinIO again and the server should now be active.

### 4: Run the IMU/GPS/WiFi synchronizer

- If the WiFi router and Raspberry Pi are connected and sending WiFi data, run the imu_gps_WiFi_publisher.py script.

- If the WiFi router and Raspberry Pi are NOT connected, run the sync_WiFi_test.py script, and in a separate terminal, run the following commands.

```
        cd ~Pathname to your WL-Local-Server repo~
        python3 WiFi_test.py
```

### 5: Run the MQTT Broker

- To show the message that is sent from the phone application to the back-end server, SSH into the WILOC server and then run the following commands:

```
        cd imu_publisher/
        mosquitto_sub -h 128.205.218.189 -t 'test/topic'
```

- To show the data that is sent back to the phone application via MQTT broker, SSH into the WILOC server and then run the following commands:

```
        cd imu_publisher/
        mosquitto_sub -h 128.205.218.189 -t 'coordinate/topic'
```

## CSI Calibration Startup

### 1: Start the MinIO Server.

1. ssh into the WILOC server:

```
        ssh wiloc@128.205.218.189
        wiloc@128.205.218.189's password: Contact Dr. Roshan Ayyalasomayajula for the server's password.

```

- When in the WILOC server, run the following command:

```
        MINIO_ROOT_USER=admin MINIO_ROOT_PASSWORD=password ./minio server /mnt/data --console-address ":9001"
```

### 2: Select the MinIO data folder

1. Go to the [MinIO Database](http://128.205.218.189:9000/).
2. Select **_wl-bucket_**.
3. Find the and copy the name of the folder you want to use (Usually ends in "\_DC").

### 3: Start the calibration script.

1. ssh into the WILOC server:

```
        ssh wiloc@128.205.218.189
        wiloc@128.205.218.189's password: Contact Dr. Roshan Ayyalasomayajula for the server's password.

```

2. cd into the **_WL-Local-Server_** repo.

```
        cd Documents/WL-Local-Server

```

3. Enable the Python virtual environment.

```
        source .venv/bin/activate
```

4. cd into the **_data_processing_** directory

```
        cd src/data_processing
```

5. Run the csi_calibration.py script.

```
        python3 csi_calibration.py
```

6. Enter the name of your data folder and the file name

```
        folder?: {Your folder name}
        filename.parquet?: {Your file name}
```

7. The CSI Heatmaps will be sent to the "{Your folder name}\_DC_Heatmaps" folder on the MinIO Server.

## ML model

### 4: Fetching parquet for ML model from MinIO database to csv. (REQUIRED FOR ML model)

1. ssh into the WILOC server:

```
        ssh wiloc@128.205.218.189
        wiloc@128.205.218.189's password: Contact Dr. Roshan Ayyalasomayajula for the server's password.

```

2. cd into the **_WL-Local-Server_** repo.

```
        cd Documents/WL-Local-Server

```

3. Enable the Python virtual environment.

```
        source .venv/bin/activate
```

4. cd into **dloc_v2**

```
        cd DLoc-cwu-fedmeta\dloc_v2
```

5. Run the .py script.

```
        python3 fetch_data_csv.py
```

6. Enter the name of your data folder.

```
        Enter MinIO folder name to index: {{Your folder name}\_DC_Heatmaps}
```

## 5: Fetching parquet for ML model from MinIO database to csv. (REQUIRED FOR ML model)

1. ssh into the WILOC server:

```
        ssh wiloc@128.205.218.189
        wiloc@128.205.218.189's password: Contact Dr. Roshan Ayyalasomayajula for the server's password.

```

2. cd into the **_WL-Local-Server_** repo.

```
        cd Documents/WL-Local-Server

```

3. Enable the Python virtual environment.

```
        source .venv/bin/activate
```

4. cd into **dloc_v2**

```
        cd DLoc-cwu-fedmeta\dloc_v2
```

5. Run the .py script.

```
        python3 fetch_data_csv.py
```

6. Enter the name of your data folder.

```
        Enter MinIO folder name to index (or 'quit' to exit): {{Your folder name}\_DC_Heatmaps}
```

7. Stop fetching

```
        Enter MinIO folder name to index (or 'quit' to exit): quit
```

## 6: Run and train the ML model

1. ssh into the WILOC server:

```
        ssh wiloc@128.205.218.189
        wiloc@128.205.218.189's password: Contact Dr. Roshan Ayyalasomayajula for the server's password.

```

2. cd into the **_WL-Local-Server_** repo.

```
        cd Documents/WL-Local-Server

```

3. Enable the Python virtual environment.

```
        source .venv/bin/activate
```

4. cd into **dloc_v2**

```
        cd DLoc-cwu-fedmeta\dloc_v2
```

5. Run the .py script.

```
        python3 main.py
```

## 7: Use the already trained model to predict location

1. ssh into the WILOC server:

```
        ssh wiloc@128.205.218.189
        wiloc@128.205.218.189's password: Contact Dr. Roshan Ayyalasomayajula for the server's password.

```

2. cd into the **_WL-Local-Server_** repo.

```
        cd Documents/WL-Local-Server

```

3. Enable the Python virtual environment.

```
        source .venv/bin/activate
```

4. cd into **dloc_v2**

```
        cd DLoc-cwu-fedmeta\dloc_v2
```

5. Run the .py script.

```
        python3 pred_loc.py
```

6. Enter the path of the parquet file

```
        Enter the path to the parquet file: 141849189164_DC_HEATMAPS/2025-04-10 15:12:12.51.parquet
```

## 8: Auto run the calibration script once where is new parquet file sent to the minio server,and also sent the predict_gps to the MQTT

1. ssh into the WILOC server:

```
        ssh wiloc@128.205.218.189
        wiloc@128.205.218.189's password: Contact Dr. Roshan Ayyalasomayajula for the server's password.

```

2. cd into the **_WL-Local-Server_** repo.

```
        cd Documents/WL-Local-Server

```

3. Enable the Python virtual environment.

```
        source .venv/bin/activate
```

4. cd into the **_data_processing_** directory

```
        cd src/data_processing
```

5. Run the watch_parquet.py script.

```
        python3 watch_parquet.py
```

6. The CSI Heatmaps will be sent to the "{Your folder name}\_DC_Heatmaps" folder on the MinIO Server and also send the predit_gps to the mqtt.

## Branch READMEs

For inquiries on the specific feature branches, check below for the following links.

1. [AoA_GroundTruth.md](/docs/AoA_GroundTruth.md)
2. [csi_calibration.md](/docs/csi_calibration.md)
3. [Data_Processing_Documentation.md](/docs/Data_Processing_Documentation.md)
4. [debugging-harry.md](/docs/debugging-harry.md)
5. [noROS_Synchronizer-Harry-Yufeng.md](/docs/noROS_Synchronizer-Harry-Yufeng.md)
6. [phone_data_collection.md](/docs/phone_data_collection.md)
7. [retrieve-specific-key-values.md](/docs/retrieve-specific-key-values.md)
8. [research-Chongze-Yang-#123.md](/docs/research-Chongze-Yang-#123.md)
9. [research-Yang-Chongze-Training-#128.md](/docs/research-Yang-Chongze-Training-#128.md)
10. [research-display-location-data-#130.md](/docs/research-display-location-data-#130.md)

## Project Roadmap

### Server-side Data Management

- [x] IMU data is received from the Android Phone via an MQTT Handler.
- [x] GPS data is received from the Android Phone via an MQTT Handler.
- [x] IMU and GPS data are synchronized within 500ms of each other.
- [x] Synchronized data can be sent to a MinIO server.
- [x] WiFi data is received from the RPI via an MQTT Handler.
- [x] WiFi data is synchronized with IMU and GPS data within 500ms of each other.
- [x] Parse the message that is sent to the MQTT broker.
- [x] Retrieve and read the parquet file based on the User ID and Timestamp that is sent from the MQTT broker.
- [x] Get the GPS Coordinates from the read parquet file and send it back to the MQTT broker.
- [x] Retrieve data for specific key requested.
