# Elasticsearch

## Introduction
This repo provides code to stream Elasticsearch on a local server and demonstrates how to interact with Elasticsearch for storing, retrieving, and managing data effectively.

## Table of Contents:
1. [Problem Definition](#problem-definition)
2. [Demographics](#demographics)
3. [Usage](#usage)
4. [Application Information](#application-information)
5. [Set Up](#set-up)

## Problem Definition
We aim to build the back-end of the Android mobile application by collecting the data from the app and pass it through a machine learning model to improve the precision of the user's location on the map. On this current sprint, we will focus on passing the following data to the MQTT server:
- User ID 
- Gyroscope readings (x,y,z-values)
- Accelerometer readings (x,y,z-values)
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

From here, the data will be queried and passed onto Elasticsearch,, which is where we can apply the machine learning model. Once the data has been passed through Elasticsearch, we will store it into MinIO. Our goal is to make sure that real-life data can be passed through MQTT, Elasticsearch, and MinIO back and forward so that the user can recieve the most accurate dataset in the application and improve the user's overall experience of using the map.

## Demographics
This application will be designed and used by University at Buffalo students and faculty, with the goal being that the application is made into an open-source platform that can be adapted for various large-scale indoor environments. 

### Usage:
This project directory consists of two nodes:

- [elastic_read](./src/elastic_read.py): This file allows the user to access the local MinIO database and insert the JSON data from MinIO to the local Elasticsearch database. In addition, this file can interact with the local Elasticsearch data base by inserting new data queries from MinIO, removing available dataset from Elasticsearch, and showing available dataset in the local Elasticsearch database.
- [minio_script](./src/minio_script.py): This file allows the user to parse the received queries from Elasticsearch and stores it in MinIO with User ID and Timestamp. In addition, the file can list all objects in the specified bucket to verify if the data has been stored or not and showing the available dataset in the local MinIO database by using User ID and Timestamp.

### Application Information:
In order for the 

### Set Up:
- To install this package, clone this repository into a directory of your choosing and install the following Debian packages from your command line:
    wget https://artifacts.elastic.co/downloads/elasticsearch/elasticsearch-8.15.2-amd64.deb
    wget https://artifacts.elastic.co/downloads/elasticsearch/elasticsearch-8.15.2-amd64.deb.sha512
    shasum -a 512 -c elasticsearch-8.15.2-amd64.deb.sha512 
    sudo dpkg -i elasticsearch-8.15.2-amd64.deb
- Next, you run Elasticsearch by using the following command lines:
    sudo /bin/systemctl daemon-reload
    sudo /bin/systemctl enable elasticsearch.service
    sudo systemctl start elasticsearch.service

### Citations:
Elasticsearch Application: https://github.com/elastic/elasticsearch
