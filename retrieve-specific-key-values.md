
WLMap Feature Branch: Retrieve-specific-key-values
Description
This branch focuses on developing functionality to retrieve specific key-value pairs from data stored in the MinIO database. The goal is to optimize the back-end server for extracting targeted information efficiently, which is essential for enhancing the indoor navigation AI models. By introducing robust querying techniques, this branch ensures that the server can handle large datasets while maintaining accuracy and speed.

This branch processes and retrieves specific data points (keys and their corresponding values) required by the AI models, enabling fine-tuned location tracking and advanced navigation features. The data is exclusively stored in Parquet files to maintain fast I/O operations and ease of access.

Key Changes
The following features are introduced in this branch:

Implementation of a feature to extract specific key-value pairs from Parquet files based on user-specified criteria.
Enhancement of server-side functionality to handle targeted data retrieval queries for improved AI model integration.
Optimization of MinIO data interaction to support real-time querying and efficient response times for large-scale datasets.
Problem Definition
This branch addresses the challenge of managing and extracting specific data elements from large-scale datasets stored in MinIO. Efficient key-value retrieval is critical for improving the indoor positioning accuracy of the mobile application.

Our primary goal is to integrate this feature into a scalable indoor navigation system designed for environments like malls, airports, and university campuses. The branch also ensures the system remains open-source and adaptable for diverse indoor navigation scenarios.

Demographics
This functionality is primarily targeted at University at Buffalo students and faculty, with a vision to expand the application's usability to large-scale public environments. The ability to extract specific data seamlessly is a vital component of the system's adaptability and scalability.

Goals and Challenges
Goals
Enable precise key-value data retrieval from Parquet files stored in MinIO.
Ensure seamless integration with back-end AI models for enhanced indoor navigation accuracy.
Develop robust querying mechanisms to handle user-specific data retrieval requests.
Maintain compatibility with the existing architecture of the WLMap application.
Challenges
Handling large-scale datasets while maintaining high-speed operations.
Managing potential data inconsistencies or missing values during key-value pair retrieval.
Ensuring compatibility with the existing indoor navigation framework.
Technology and Development Plans
Technologies
Kotlin: For Android application development and integration.
MinIO: As the primary database for storing Parquet files.
Python: For back-end processing and AI model integration.
PySpark & Pandas: For efficient data manipulation and processing.
Development Plans
Review and understand Parquet file structures for key-value extraction.
Develop Python functions to retrieve specific key-value pairs from Parquet files.
Optimize retrieval functions using PySpark for large-scale data processing.
Test server-side functionality to ensure seamless integration with the WLMap application.
Features
So far, this branch has implemented the following features:

Query and extract specific key-value pairs from Parquet files.
Handle user-defined criteria for targeted data retrieval.
Ensure compatibility with data processing frameworks (Pandas and PySpark).
Optimize server-side data queries for real-time responsiveness.
Tools
The following tools are utilized for development and deployment:

Kotlin and Android Studio: For mobile application development.
Python: For back-end server development.
Pandas & PySpark: For data manipulation and processing.
MinIO: As the primary database for storing Parquet files.
MQTT: For message parsing and server-client communication.
Deployment Instructions
Android Studio Setup
Download and Install Android Studio

For Mac users: Download the appropriate installer based on your chipset (Intel Chip or Apple Chip).
Follow the installer instructions during installation.
Ensure compatibility with the Pixel 7a (API 35).
Clone the Repository

bash
Copy code
git clone https://github.com/WS-UB/WL-Local-Server.git
Checkout the Feature Branch

sql
Copy code
git checkout feature/Retrieve-specific-key-values
Update AGP (Android Gradle Plugin)

Follow the steps outlined in the "AGP Upgrade Assistant" to upgrade to version 8.6.
Set Up Device Emulator

Create a virtual device (Pixel 7a, API 35) in Android Studio.
Launch the emulator and open the WLMap application.
Back-End Server Setup
Install the required Python libraries:

Copy code
pip3 install minio urllib3 paho-mqtt pyspark pandas pyarrow
Run the following commands to execute back-end functions:

php
Copy code
python3 <file_name>.py
Replace <file_name> with the appropriate Python script located in the cloned repository.

Project Roadmap
Data Retrieval
Implement and test functions for retrieving specific key-value pairs from Parquet files.
Ensure compatibility with the existing data filtering and cleaning pipeline.
Integration with MQTT
Enable real-time data transfer of retrieved key-value pairs to MQTT subscribers.
AI Model Training
Use extracted key-value data to refine and enhance AI model training for indoor navigation.
Testing and Debugging
Conduct rigorous testing of the retrieval feature to ensure accuracy and efficiency.
Address any bugs or performance issues during the integration process.
