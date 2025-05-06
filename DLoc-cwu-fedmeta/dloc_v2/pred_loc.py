print("================================================================================================================")
import string
import json
import sys
import random
import os
os.chdir('/home/wiloc/Documents/WL-Local-Server/DLoc-sp25-cse302/dloc_v2')
from model import TrigAOAResNetModel
from dataset import DLocDatasetV2
from torch.utils.data import DataLoader
from gps_cali import pred_reverse_normalization
from pathlib import Path
# Go up from `dloc_v2` to `DLoc-owl-fedmeta`, then into `src`
project_root = Path(__file__).resolve().parent.parent.parent  # Adjust based on actual structure
src_path = str(project_root / "src")  # Path to `src` folder
sys.path.append(src_path)
from MQTT_Handler import MQTTHandler  # Adjust import based on actual structure
import random
import time
from decimal import Decimal, getcontext
getcontext().prec = 25  # Set precision for all Decimal operations


# MQTT Configuration
MQTT_BROKER = "128.205.218.189"  # Same as in nexcsiserver.py
MQTT_PORT = 1883
PREDICTION_TOPIC = "/predicted_location"

def predict_gps(parquet_file_path):
    """
    Predict GPS coordinates from a parquet file using a trained model.
    
    Args:
        parquet_file_path: Path to the parquet file containing heatmap data
    """
    # Connect to MQTT broker
    client_id = f"pred_loc_{random.randint(0, 1000)}"  # Random client ID
    mqtt_handler = MQTTHandler(client_id, MQTT_BROKER, MQTT_PORT, PREDICTION_TOPIC)
    mqtt_handler.client = mqtt_handler.connect_mqtt()
    mqtt_handler.client.loop_start()  # Start network loop


    # Load the trained model
    model = TrigAOAResNetModel.load_from_checkpoint("saved_models/best_model.ckpt")

    # Load the dataset and create a DataLoader
    test_dataset = DLocDatasetV2(parquet_path=parquet_file_path)
    test_loader = DataLoader(test_dataset, batch_size=1)

    # Run prediction
    model.eval()
    for heatmap, _, _ in test_loader:
        output = model(heatmap)
        norm_lat = Decimal(str(output.location[0][0].item()))
        norm_lon = Decimal(str(output.location[0][1].item()))
        
        # High-precision denormalization
        lat, lon = pred_reverse_normalization(
            norm_lat, norm_lon
        )
        
        # Format to 20 decimal places
        lat_str = format(lat, '.20f')
        lon_str = format(lon, '.20f')
        print(f"Predicted GPS: {lat_str}, {lon_str}")
        print(f"normalized GPS: {norm_lat}, {norm_lon}")
        location_data = {
            "latitude": lat_str,
            "longitude": lon_str,
            "source": "pred_loc.py",
            "precision": "20 decimal places"
        }
        mqtt_handler.publish(mqtt_handler.client, json.dumps(location_data))



# testing the function with a sample parquet file
if __name__ == "__main__":
    # # # (1) Load the trained model
    # model = TrigAOAResNetModel.load_from_checkpoint("saved_models/best_model.ckpt")

    # # Load a single parquet file
    # test_dataset = DLocDatasetV2(parquet_path="866469054878_DC_HEATMAPS/2025-04-23 12:14:08.492.parquet")
    # test_loader = DataLoader(test_dataset, batch_size=1)

    # # Run prediction
    # model.eval()
    # # model = model.to(dev)
    # for heatmap, _, _ in test_loader:
    #     output = model(heatmap)
    #     denormalized_output = reverse_normalization(output.location[0][0], output.location[0][1])
    #     loc = denormalized_output[0].item(), denormalized_output[1].item()
    #     print("Predicted GPS:", loc)
    file_path = input("Enter the path to the parquet file: ")

    # client_id = f"pred_loc_{random.randint(0, 1000)}"  # Random client ID
    # mqtt_handler = MQTTHandler(client_id, MQTT_BROKER, MQTT_PORT, PREDICTION_TOPIC)
    # mqtt_handler.client = mqtt_handler.connect_mqtt()
    # mqtt_handler.client.loop_start()  # Start network loop

    predict_gps(file_path)