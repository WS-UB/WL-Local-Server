import os
import pandas as pd
import numpy as np
from datetime import datetime
from minio import Minio
from dotenv import load_dotenv

def get_minio_client():
    """Initialize and return a MinIO client using environment variables."""
    load_dotenv()
    return Minio(
        os.getenv("MINIO_ENDPOINT"),
        access_key=os.getenv("MINIO_ACCESS_KEY"),
        secret_key=os.getenv("MINIO_SECRET_KEY"),
        secure=os.getenv("MINIO_SECURE", "false").lower() == 'true',
    )

def get_parquet_files(minio_client, bucket_name, folder):
    """Get all parquet files in the specified folder from MinIO."""
    prefix = f"{folder}/"
    objects = list(minio_client.list_objects(bucket_name, prefix=prefix, recursive=True))
    return [obj for obj in objects if obj.object_name.endswith(".parquet")]

def create_file_metadata(parquet_files, folder):
    """Create a DataFrame with just the columns we care about."""
    records = []
    for obj in parquet_files:
        records.append({
            "folder": folder,
            "file_name": obj.object_name,
            "file_path": obj.object_name,
        })
    return pd.DataFrame(records)

def split_and_save_data(metadata_df, base_path="data"):
    """
    Split data into train (70%), test (20%), and validation (10%) sets
    and save them as separate CSV files.
    """
    try:
        # Ensure data directory exists
        os.makedirs(base_path, exist_ok=True)
        
        # Shuffle the data
        shuffled_df = metadata_df.sample(frac=1.0, random_state=42)
        
        # Calculate split sizes
        total_size = len(shuffled_df)
        train_size = int(0.7 * total_size)
        test_size = int(0.2 * total_size)
        
        # Split the data
        train_df = shuffled_df.iloc[:train_size]
        test_df = shuffled_df.iloc[train_size:train_size + test_size]
        validation_df = shuffled_df.iloc[train_size + test_size:]

        # Reset & name the integer index (starts at 1)
        def prepare_and_save_df(df, name):
            # keep only the 3 data columns
            df = df[["folder", "file_name", "file_path"]]
            
            # reset & name the integer index (starts at 1)
            df.reset_index(drop=True, inplace=True)
            df.index = df.index + 1
            df.index.name = "index"
            
            # Save to CSV
            csv_path = os.path.join(base_path, f"{name}_index.csv")
            df.to_csv(csv_path, index=True)
            print(f"✅ {name.capitalize()} CSV created at {csv_path}, {len(df)} records.")
            return csv_path
        
        # Save all three datasets
        train_path = prepare_and_save_df(train_df, "train")
        test_path = prepare_and_save_df(test_df, "test")
        val_path = prepare_and_save_df(validation_df, "validation")
        
        print(f"\nData split successfully: {len(train_df)} train, {len(test_df)} test, {len(validation_df)} validation")
        print(f"Split ratio: {len(train_df)/total_size:.1%}/{len(test_df)/total_size:.1%}/{len(validation_df)/total_size:.1%}")
        
        return True

    except Exception as e:
        print(f"Error splitting and saving data: {e}")
        return False

def main(bucket_name="wl-data"):
    try:
        minio_client = get_minio_client()
        folder = input("Enter MinIO folder name to index: ").strip()
        parquet_files = get_parquet_files(minio_client, bucket_name, folder)
        if not parquet_files:
            print(f"⚠️ No parquet files found in '{folder}'.")
            return

        metadata_df = create_file_metadata(parquet_files, folder)

        # Instead of updating a single CSV, split and save as train/test/validation
        split_and_save_data(metadata_df)
        
        # Optional: Also save the full dataset as before
        full_csv_path = os.path.join("data", "parquet_index.csv")
        full_df = metadata_df[["folder", "file_name", "file_path"]]
        full_df.reset_index(drop=True, inplace=True)
        full_df.index = full_df.index + 1
        full_df.index.name = "index"
        full_df.to_csv(full_csv_path, index=True)
        print(f"✅ Full dataset CSV saved at {full_csv_path}, {len(full_df)} records.")

    except Exception as e:
        print(f"❌ Error: {e}")


if __name__ == "__main__":
    main()