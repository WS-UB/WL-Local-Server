import os
import pandas as pd
import numpy as np
from datetime import datetime
from minio import Minio
from dotenv import load_dotenv
from pathlib import Path

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

def load_or_create_csv(csv_path):
    """Load existing CSV or create empty DataFrame if file doesn't exist."""
    if Path(csv_path).exists():
        df = pd.read_csv(csv_path, index_col="index")
        # Ensure index starts from 1 (in case empty df was saved)
        if not df.empty:
            df.index = range(1, len(df) + 1)
        return df
    return pd.DataFrame(columns=["folder", "file_name", "file_path"])

def prepare_and_save_df(df, csv_path):
    """Prepare dataframe with proper indexing and save to CSV."""
    df = df[["folder", "file_name", "file_path"]].copy()
    df.reset_index(drop=True, inplace=True)
    df.index = df.index + 1  # Start index at 1
    df.index.name = "index"
    df.to_csv(csv_path, index=True)
    return df

def update_datasets(new_df, base_path="data"):
    """
    Update train, test, validation datasets with new data, avoiding duplicates.
    If datasets don't exist, create new ones.
    Maintains separate indexing for each dataset starting at 1.
    """
    try:
        # Ensure data directory exists
        os.makedirs(base_path, exist_ok=True)
        
        # Define dataset paths
        dataset_paths = {
            "train": os.path.join(base_path, "train_index.csv"),
            "test": os.path.join(base_path, "test_index.csv"),
            "validation": os.path.join(base_path, "validation_index.csv"),
            "full": os.path.join(base_path, "parquet_index.csv")
        }
        
        # Load or create each dataset
        datasets = {name: load_or_create_csv(path) for name, path in dataset_paths.items()}
        
        # For full dataset, check for duplicates and append new data
        full_df = datasets["full"]
        if not full_df.empty:
            # Filter out files that already exist in the full dataset
            existing_files = set(full_df["file_path"])
            new_df = new_df[~new_df["file_path"].isin(existing_files)]
            
            if new_df.empty:
                print("ℹ️ All files from this folder are already indexed. No updates needed.")
                return False
        
        # If we have new data to add
        if not new_df.empty:
            # Update full dataset
            updated_full = pd.concat([full_df, new_df], ignore_index=True)
            updated_full = prepare_and_save_df(updated_full, dataset_paths["full"])
            print(f"✅ Updated full dataset CSV with {len(new_df)} new records. Total: {len(updated_full)}")
            
            # Reshuffle the FULL dataset (not just new data) to maintain proper ratios
            shuffled_df = updated_full.sample(frac=1.0, random_state=42)
            total_size = len(shuffled_df)
            train_size = int(0.7 * total_size)
            test_size = int(0.2 * total_size)
            
            # Split the data
            train_df = shuffled_df.iloc[:train_size]
            test_df = shuffled_df.iloc[train_size:train_size + test_size]
            validation_df = shuffled_df.iloc[train_size + test_size:]
            
            # Prepare and save each dataset with independent indexing
            prepare_and_save_df(train_df, dataset_paths["train"])
            prepare_and_save_df(test_df, dataset_paths["test"])
            prepare_and_save_df(validation_df, dataset_paths["validation"])
            
            print(f"\n✅ Datasets updated successfully:")
            print(f"- Train: {len(train_df)} records (index 1-{len(train_df)})")
            print(f"- Test: {len(test_df)} records (index 1-{len(test_df)})")
            print(f"- Validation: {len(validation_df)} records (index 1-{len(validation_df)})")
            print(f"Split ratio: {len(train_df)/total_size:.1%}/{len(test_df)/total_size:.1%}/{len(validation_df)/total_size:.1%}")
            
            return True
        
        return False

    except Exception as e:
        print(f"❌ Error updating datasets: {e}")
        return False

def process_folder(minio_client, bucket_name, folder):
    """Process a single folder and return new metadata DataFrame."""
    parquet_files = get_parquet_files(minio_client, bucket_name, folder)
    if not parquet_files:
        print(f"⚠️ No parquet files found in '{folder}'.")
        return None
    
    return create_file_metadata(parquet_files, folder)

def main(bucket_name="wl-data"):
    try:
        minio_client = get_minio_client()
        
        while True:
            folder = input("\nEnter MinIO folder name to index (or 'quit' to exit): ").strip()
            if folder.lower() in ('quit', 'exit', 'q'):
                break
                
            new_df = process_folder(minio_client, bucket_name, folder)
            if new_df is not None:
                update_datasets(new_df)
                
        print("\nIndexing complete. CSV files are up to date.")

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()