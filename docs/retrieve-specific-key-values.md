Description
This branch focuses on implementing functionality to retrieve specific key-value pairs from data stored in the MinIO database. The aim is to optimize the back-end server for targeted information extraction, which is crucial for enhancing the AI models used in indoor navigation systems. The data is stored in Parquet files to facilitate fast I/O operations, supporting the system's need for rapid data retrieval and efficient processing.

Key features in this branch ensure that the server can handle real-time queries and large datasets efficiently while maintaining accuracy. This feature is vital for improving the indoor positioning accuracy of mobile applications, such as those used in large public spaces like malls, airports, and campuses.

Key Changes
The branch introduces the following features:

Automated Input Parsing: A function to handle the parsing of user input, identifying user IDs, timestamps, and keys for data retrieval.
Data Extraction: Extraction of key-value pairs from Parquet files, including support for nested data structures (JSON, dictionaries).
Optimized Data Retrieval: Integration with MinIO to retrieve specific data points from Parquet files efficiently, supporting real-time querying for the AI models.
Problem Definition
The main challenge addressed by this branch is efficiently managing and retrieving specific data points from large-scale datasets stored in MinIO. Real-time data retrieval is crucial for enhancing the indoor positioning accuracy of mobile applications, especially in environments like airports and shopping malls.

The goal is to integrate this feature into the WLMap system, ensuring it supports scalable indoor navigation for various environments while maintaining system flexibility and open-source adaptability.

Demographics
Target Users: University at Buffalo students and faculty, with plans to expand to large-scale public environments (malls, airports, campuses).
Use Case: Enhancing indoor navigation features through efficient key-value data extraction, improving overall system performance and scalability.
Goals and Challenges
Goals:

Enable efficient key-value data retrieval from Parquet files stored in MinIO.
Integrate with AI models for enhanced indoor navigation accuracy.
Support real-time, user-specific data queries for large datasets.
Challenges:

Handling large-scale datasets without compromising speed.
Ensuring data integrity and consistency, especially with nested or missing values.
Maintaining seamless integration with the existing indoor navigation framework.
Technology and Development Plans
Technologies:

Kotlin: For Android application development and integration.
MinIO: For storing Parquet files.
Python: For back-end server processing and AI model integration.
PySpark & Pandas: For efficient data manipulation and processing.
Development Plans:

Develop and test functions for retrieving specific key-value pairs from Parquet files stored in MinIO.
Optimize data retrieval functions using PySpark for better performance with large datasets.
Integrate the back-end server with the WLMap application for seamless communication.
Features Implemented So Far
Automated Input Parsing: Parsing of input lists to extract user IDs, timestamps, and keys for targeted data retrieval.
Key-Value Data Retrieval: Extraction of key-value pairs from Parquet files using pandas and handling of nested data structures.
MinIO Integration: Optimized querying and retrieval of data from Parquet files stored in MinIO.
Tools Used for Development
Kotlin & Android Studio: For mobile application development.
Python: For back-end server processing.
MinIO: As the primary database for storing Parquet files.
PySpark & Pandas: For efficient data manipulation and processing.
MQTT: For server-client communication and data transfer.
Deployment Instructions
Android Studio Setup:

Download and install Android Studio (choose the appropriate installer for your chipset).
Clone the repository:
bash
Copy code
git clone https://github.com/WS-UB/WL-Local-Server.git
Checkout the feature branch:
sql
Copy code
git checkout feature/Retrieve-specific-key-values
Set up your Android Emulator (Pixel 7a, API 35) and launch the WLMap application.
Back-End Server Setup:

Install the required Python libraries:
bash
Copy code
pip3 install minio urllib3 paho-mqtt pyspark pandas pyarrow
Run the Python script:
bash
Copy code
python3 <file_name>.py
Project Roadmap
Data Retrieval: Implement and test functions for retrieving specific key-value pairs from Parquet files.
Integration with MQTT: Enable real-time data transfer of extracted data to MQTT subscribers.
AI Model Training: Use retrieved data to improve AI model training for better indoor navigation.
Testing & Debugging: Rigorous testing of data retrieval functionalities to ensure accuracy and performance.
Code Integration Overview
Automated Input Parsing: The parse_automated_input() function processes the input list, extracting user IDs, timestamps, and keys for subsequent data retrieval.

Key-Value Extraction: The extract_key_data() function handles extraction from Parquet files, including support for nested data formats like JSON and dictionaries.

Data Retrieval: The retrieve_data() function connects to MinIO, retrieves Parquet files for the given user IDs and timestamps, and extracts requested key-value pairs.

Automated Query Handling: The handle_automated_query() function facilitates automated query processing and integrates all components, providing a list of DataFrames with the retrieved data.

