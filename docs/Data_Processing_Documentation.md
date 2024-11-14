# WLMap Feature Branch: wifi-data-processing

## Description

This branch is about implementing various data manipulation and filtering techniques so that it can be used for various AI model. For context, this branch will be receving and reading data from the MinIO database and make some changes to those data so that it can be passed through various filtering function and AI model for better location tracking functionality. In addition, the database will be storing exclusively parquet files only for faster readability and convenient retrieving. The goal of this branch is to process data through filtering, cleaning, and manipulating so that it can be passed through the requiments of various AI models.   

## Key Changes
This feature introduces the following changes:
  - Implementing a feature that can insert a new data item in a set of given parquet files
  - Implementing a feature that can retrieve a set of parquet files based on the chosen range of hour and minute.

## Problem Definition

We aim to create a stable Android mobile application for indoor navigation and WiFi-based data collection. This semester, we will focus on resolving technical issues with the data collection app, enhancing the user interface, and integrating server-side data processing using AWS. Additionally, we will develop a functional navigational interface similar to Google Maps, enabling users to track their indoor location within large buildings like malls and airports. In the long term, we aspire to deploy a fully functional, scalable system that enables seamless indoor navigation by utilizing WiFi signals and real-time data collection. By leveraging machine learning models, we will enhance accuracy in indoor positioning, ensuring privacy and efficiency through the use of hashed user data. Our goal is to provide a robust and open-source platform that can be adapted for various large-scale indoor environments.

## Demographics

This application will be designed and used by University at Buffalo students and faculty, with the goal being that the application is made into an open-source platform that can be adapted for various large-scale indoor environments. 

## Goals and Challenges
Our current goals for this project include:
   - Storing user/device data using server integration.
   - Retrieving server-side data to update user position.
   - Gather and store user navigation data in MinIO.
   - Integrating data processing techniques using Pandas and PySpark.
   - Use stored reliable data for an A.I. training model.


## Technology and Development Plans
Our current technology and development plans include:
   - Kotlin: Watch video tutorials and read documentation for better understanding.
   - UI/UX design: Watch video tutorials and refer to Figma UI outline as a reference for design features.
   - MQTT: Read MQTT documentation and refer to Dr. Roshan for better understanding and implementation.
   - Pandas & PySpark: Read Pandas & PySpark documentation online



## Features
So far, this branch has implemented the following features
   - Read and parse messages sent from MQTT broker
   - Collecting data via RaspberryPi, Wifi routers, and Google Pixel 7a phone.
   - Read and modify data by using various imported functions from Panda and PySpark


## Tools
Kotlin and Android Studio are used to create the application on the Android platform, more specifically, a Google Pixel 7A.
Python is mostly used for collecting, processing, and passing through AI models in the back-end server.

## Deployment Instructions

1. Download and install [Android Studio](https://developer.android.com/studio)
   - For Mac users: Download the appropriate installer based on your chipset (Intel Chip or Apple Chip).
   - Follow installer instructions during installation.
   - This app is compatible with the Pixel 7a (API 35).

2. Clone the appropriate repository.

3. Allow Gradle to install and update the AGP (Android Gradle Plugin) to version 8.6 if prompted
   - If you are not prompted to update the AGP, follow these instructions:
   - Select the "Tools" drop-down menu on the top of the IDE.
    - Select "AGP Upgrade Assistant."
     - Select version 8.6.
     - Select "Run selected steps."
     - After the update is complete, select "Refresh."

4. Select "Device Manager" on the right-side app bar and install the device emulator.
   - If there are any other created devices, end their processes and remove them.
   - Click the "+" to and select "Create Virtual Device."
   - Select the "Phone" category and select the Pixel 7a (API 35).
   - Press "Finish."
   - After the device is installed, select the "play" button next to the installed device to begin running it.
  
5. In your terminal, cd into the directory of your WL-Local-Server repository and type the following command.

```
   git checkout feature/wifi-data-processing
```

6. Open and run the WLMap application in Android Studio, selecting the "Navigation" option on the home screen of WLMap.

7. On your terminal, import all of the following libraries so that the code can be ran

```
    pip3 install minio urllib3 paho-mqtt pyspark pandas pyarrow
```

8. Now, we run the following commands to run any functions in the back-end
```
   python3 <file_name>.py
```
Note: The file name must be located in the cloned repository



## Project Roadmap

### Data Readings sent to MQTT client
- [x] GPS data is sent to MQTT server.
- [x] Accelerometer & gyroscope data is sent to MQTT broker.
- [x] User ID & timestamp is sent to MQTT broker.
- [x] Accelerometer, gyroscope, user ID and timestamp is received on the back-end server via MQTT subscriber.

### Process Data accordingly
- [x] Retrieve and read data from MinIO database.
- [x] Get data from MinIO based on given timestamp.
- [x] Retrieve certain sets of data from MinIO based on chosen range of hours and minute.
- [x] Add new data item to a chosen list of data.
- [] Retrieve certain column of data in a list of dataframes.