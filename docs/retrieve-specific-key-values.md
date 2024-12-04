# WL-Local-Server branch Retrieve-specific-key-values

## Description

This branch focuses on implementing functionality to retrieve specific key-value pairs from data stored in the MinIO database. The aim is to optimize the back-end server for targeted information extraction, which is crucial for enhancing the AI models used in indoor navigation systems. The data is stored in Parquet files to facilitate fast I/O operations, supporting the system's need for rapid data retrieval and efficient processing.

## Key Features
Key features in this branch ensure that the server can handle real-time queries and large datasets efficiently while maintaining accuracy. This feature is vital for improving the indoor positioning accuracy of mobile applications, such as those used in large public spaces like malls, airports, and campuses.

## Key Changes

The branch introduces the following features:

   -Automated Input Parsing: A function to handle the parsing of user input, identifying user IDs, timestamps, and keys for data retrieval.

   -Data Extraction: Extraction of key-value pairs from Parquet files, including support for nested data structures (JSON, dictionaries).

   -Optimized Data Retrieval: Integration with MinIO to retrieve specific data points from Parquet files efficiently, supporting real-time querying for the AI models.


## Problem Definition

The main challenge addressed by this branch is efficiently managing and retrieving specific data points from large-scale datasets stored in MinIO. Real-time data retrieval is crucial for enhancing the indoor positioning accuracy of mobile applications, especially in environments like airports and shopping malls.

The goal is to integrate this feature into the WLMap system, ensuring it supports scalable indoor navigation for various environments while maintaining system flexibility and open-source adaptability.

## Demographics

Target Users: University at Buffalo students and faculty, with plans to expand to large-scale public environments (malls, airports, campuses).
Use Case: Enhancing indoor navigation features through efficient key-value data extraction, improving overall system performance and scalability.

## Goals and Challenges

### Goals:
  -Enable efficient key-value data retrieval from Parquet files stored in MinIO.
  -Integrate with AI models for enhanced indoor navigation accuracy.
  -Support real-time, user-specific data queries for large datasets.
### Challenges:
  -Handling large-scale datasets without compromising speed.
  -Ensuring data integrity and consistency, especially with nested or missing values.
  -Maintaining seamless integration with the existing indoor navigation framework.
  -Technology and Development Plans

## Technologies:

  -Kotlin: For Android application development and integration.
  -MinIO: For storing Parquet files.
  -Python: For back-end server processing and AI model integration.
  -PySpark & Pandas: For efficient data manipulation and processing.

## Development Plans:

Develop and test functions for retrieving specific key-value pairs from Parquet files stored in MinIO.
Optimize data retrieval functions using PySpark for better performance with large datasets.
Integrate the back-end server with the WLMap application for seamless communication.
Features Implemented So Far
Automated Input Parsing: Parsing of input lists to extract user IDs, timestamps, and keys for targeted data retrieval.
Key-Value Data Retrieval: Extraction of key-value pairs from Parquet files using pandas and handling of nested data structures.
MinIO Integration: Optimized querying and retrieval of data from Parquet files stored in MinIO.

## Tools Used for Development
Kotlin & Android Studio: For mobile application development.
Python: For back-end server processing.
MinIO: As the primary database for storing Parquet files.
PySpark & Pandas: For efficient data manipulation and processing.
MQTT: For server-client communication and data transfer.

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

Project Roadmap
- [x]Data Retrieval: Retrieves specific key-value pairs from Parquet files stored in MinIO.
- [x]Integration with MQTT: Enables real-time data transfer of extracted data to MQTT subscribers.
- [x]Automated Input Parsing: Processes input lists to extract user IDs, timestamps, and keys for data retrieval.
- [x]Key-Value Extraction: Extracts key-value pairs from Parquet files, including nested formats like JSON.
- [x]Parquet Data Retrieval: Connects to MinIO, retrieves Parquet files, and extracts requested key-value pairs.
- [x]Automated Query Handling: Facilitates automated queries and integrates components to return DataFrames.

