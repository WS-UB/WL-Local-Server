# MinIO Database

## Introduction:
This repository contains Python scripts and configurations to interact with MinIO, store data efficiently in Parquet format, and query stored data via Elasticsearch. The system also features an MQTT subscriber for real-time data ingestion. It is built to handle incoming sensor data streams and store them in MinIO for further analysis.


## Table of Contents:
1. [Overview](#overview)
1. [Usage](#usage)
2. [Set Up](#set-up)
3. [Citations](#citations)

### Overview:
This project demonstrates how to manage and store real-time IoT data streams using MinIO and Parquet format. It also includes integration with Elasticsearch to query stored data and uses MQTT to receive sensor data.

The key objectives are:

Efficient data storage using MinIO with Parquet.
Real-time data ingestion via MQTT.
Query and analysis using Elasticsearch.
Docker support for easy deployment.

### Usage:
minio_script: This file allows the user to parse the received queries from Elasticsearch and store them in MinIO with User ID and Timestamp. In addition, the file can list all objects in the specified bucket to verify if the data has been stored or not and showing the available dataset in the local MinIO database by using User ID and Timestamp.

