import os
import pandas as pd
from io import StringIO
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
    parquet_files = [obj for obj in objects if obj.object_name.endswith(".parquet")]
    return parquet_files

def create_file_metadata(parquet_files, folder):
    """Create a DataFrame with metadata about the parquet files."""
    data = []
    for obj in parquet_files:
        file_name = obj.object_name
        file_size_mb = obj.size / (1024 * 1024)  # Convert bytes to MB
        
        data.append({
            "folder": folder,
            "file_name": file_name,
            "file_path": file_name,
            "size_mb": round(file_size_mb, 2),
            "last_modified": obj.last_modified,
            "indexed_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })
    
    return pd.DataFrame(data)

def update_csv_index(new_data, csv_path="parquet_index.csv"):
    """Update the CSV index with new parquet file information,
    and write its integer index as the first column."""
    try:
        # Load existing index if any
        if os.path.exists(csv_path):
            existing_data = pd.read_csv(csv_path)
            folder = new_data["folder"].iloc[0]
            existing_data = existing_data[existing_data["folder"] != folder]
            combined_data = pd.concat([existing_data, new_data], ignore_index=True)
        else:
            combined_data = new_data

        # --- NEW: reset and name the integer index, so it becomes a column ---
        combined_data.reset_index(drop=True, inplace=True)        # ensure clean 0-based index
        combined_data.index = combined_data.index + 1             # if you want to start at 1
        combined_data.index.name = "index"                        # give it a column name

        # now write it out, index=True writes that “index” column first
        combined_data.to_csv(csv_path, index=True)

        print(f"Successfully updated index with {len(new_data)} new records.")
        print(f"Total records in index: {len(combined_data)}")
        return True

    except Exception as e:
        print(f"Error updating CSV index: {e}")
        return False

def main(bucket_name="wl-data"):
    """Main function to execute the script."""
    try:
        minio_client = get_minio_client()

        folder = input("Enter MinIO folder name to index: ").strip()
        parquet_files = get_parquet_files(minio_client, bucket_name, folder)
        if not parquet_files:
            print(f"⚠️ No parquet files found in '{folder}'.")
            return

        metadata_df = create_file_metadata(parquet_files, folder)

        csv_folder = "data"
        csv_path = os.path.join(csv_folder, "parquet_index.csv")


        os.makedirs(csv_folder, exist_ok=True)

        update_csv_index(metadata_df, csv_path)
        print("\n✅ Indexing complete!")

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()