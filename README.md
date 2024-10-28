MinIO Database

## Introduction:
This repository contains Python scripts and configurations to interact with MinIO, store data efficiently in Parquet format, and query stored data via Elasticsearch. The system also features an MQTT subscriber for real-time data ingestion. It is built to handle incoming sensor data streams and store them in MinIO for further analysis.

Table of Contents
Overview
Features
Prerequisites
Setup
Usage
File Structure
Testing
Docker Support
License

Overview
This project demonstrates how to manage and store real-time IoT data streams using MinIO and Parquet format. It also includes integration with Elasticsearch to query stored data and uses MQTT to receive sensor data.

The key objectives are:

Efficient data storage using MinIO with Parquet.
Real-time data ingestion via MQTT.
Query and analysis using Elasticsearch.
Docker support for easy deployment.

Features
MinIO Storage: Store structured data in MinIO in Parquet format.
Elasticsearch Queries: Query stored data for quick lookups.
MQTT Subscriber: Ingest data from IoT devices in real-time.
Python Automation: Automate data storage and retrieval with Python scripts.


bash
Copy code
python minio_load.py

