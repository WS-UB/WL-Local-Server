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
    """Update the CSV index with new parquet file information."""
    try:
        # Check if the CSV file exists
        if os.path.exists(csv_path):
            # Read existing CSV
            existing_data = pd.read_csv(csv_path)
            print(f"Loaded existing index with {len(existing_data)} records.")
            
            # Remove entries from the same folder if they exist
            folder = new_data["folder"].iloc[0]
            existing_data = existing_data[existing_data["folder"] != folder]
            print(f"Removed old entries for folder '{folder}'.")
            
            # Concatenate with new data
            combined_data = pd.concat([existing_data, new_data], ignore_index=True)
        else:
            # If file doesn't exist, use only the new data
            combined_data = new_data
            print(f"Creating new index file at {csv_path}")
        
        # Save the updated dataframe to CSV
        combined_data.to_csv(csv_path, index=False)
        print(f"Successfully updated index with {len(new_data)} new records.")
        print(f"Total records in index: {len(combined_data)}")
        return True
    
    except Exception as e:
        print(f"Error updating CSV index: {e}")
        return False

def main(bucket_name="wl-data"):
    """Main function to execute the script."""
    try:
        # Initialize MinIO client
        minio_client = get_minio_client()
        
        # Get folder name from user
        folder = input("Enter MinIO folder name to index: ").strip()
        
        # Get all parquet files in the folder
        print(f"\nListing parquet files in '{folder}'...")
        parquet_files = get_parquet_files(minio_client, bucket_name, folder)
        
        if not parquet_files:
            print(f"⚠️ No parquet files found in '{folder}'.")
            return
        
        # Display found files
        print(f"\nFound {len(parquet_files)} parquet files:")
        for i, obj in enumerate(parquet_files[:5]):  # Show first 5 files
            print(f"  • {obj.object_name} ({round(obj.size / (1024 * 1024), 2)} MB)")
        
        if len(parquet_files) > 5:
            print(f"  • ... and {len(parquet_files) - 5} more files")
        
        # Create metadata DataFrame
        metadata_df = create_file_metadata(parquet_files, folder)
        
        # Get CSV path
        csv_path = input("\nEnter path for the CSV index file (default: parquet_index.csv): ").strip()
        if not csv_path:
            csv_path = "parquet_index.csv"
        
        # Update the CSV index
        update_csv_index(metadata_df, csv_path)
        
        print("\n✅ Indexing complete!")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()