import os
import pandas as pd
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

def update_csv_index(new_data, csv_path="data/parquet_index.csv"):
    """Rewrite the CSV so it only has index, folder, file_name, file_path."""
    try:
        # Load existing or start fresh
        if os.path.exists(csv_path):
            existing = pd.read_csv(csv_path)
            # drop any old entries from this folder
            folder = new_data["folder"].iloc[0]
            existing = existing[existing["folder"] != folder]
            combined = pd.concat([existing, new_data], ignore_index=True)
        else:
            combined = new_data

        # keep only the 3 data columns
        combined = combined[["folder", "file_name", "file_path"]]

        # reset & name the integer index (starts at 1 here)
        combined.reset_index(drop=True, inplace=True)
        combined.index = combined.index + 1
        combined.index.name = "index"

        # write it out—index=True makes that first column
        combined.to_csv(csv_path, index=True)
        print(f"✅ CSV updated at {csv_path}, {len(combined)} total records.")
        return True

    except Exception as e:
        print(f"Error updating CSV index: {e}")
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

        # ensure `data/` exists
        csv_folder = "data"
        os.makedirs(csv_folder, exist_ok=True)
        csv_path = os.path.join(csv_folder, "parquet_index.csv")

        update_csv_index(metadata_df, csv_path)

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()