import os
import random
import pandas as pd
from io import BytesIO
from concurrent.futures import ThreadPoolExecutor
from minio import Minio
from dotenv import load_dotenv

def fetch_parquet_data(minio_client, bucket_name, obj):
    """
    Fetch a single object from MinIO and return as pandas DataFrame.
    """
    try:
        print(f"Fetching {obj.object_name}")
        response = minio_client.get_object(bucket_name, obj.object_name)
        data = pd.read_parquet(BytesIO(response.read()), engine="pyarrow")
        print(f"Successfully fetched {obj.object_name}")
        return data
    except Exception as e:
        print(f"Failed to fetch {obj.object_name}: {e}")
        return None

def fetch_files_parallel(file_list, minio_client, bucket_name, max_workers=8):
    """
    Fetch a list of files in parallel using ThreadPoolExecutor.
    Returns a list of pandas DataFrames.
    """
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        results = list(executor.map(
            lambda obj: fetch_parquet_data(minio_client, bucket_name, obj), file_list))
    dataframes = [df for df in results if df is not None]
    return dataframes

def fetch_selected_parquet_from_minio(bucket_name="wl-data"):
    """
    Prompts for a MinIO folder and lists all .parquet files.
    Allows the user to choose one file to fetch and process,
    and then returns the data as a pandas DataFrame.
    """
    load_dotenv()
    minio_client = Minio(
        os.getenv("MINIO_ENDPOINT"),
        access_key=os.getenv("MINIO_ACCESS_KEY"),
        secret_key=os.getenv("MINIO_SECRET_KEY"),
        secure=os.getenv("MINIO_SECURE").lower() == 'true',
    )
    folder = input("Enter MinIO folder name: ").strip()
    prefix = f"{folder}/"
    
    try:
        objects = list(minio_client.list_objects(bucket_name, prefix=prefix, recursive=True))
        parquet_files = [obj for obj in objects if obj.object_name.endswith(".parquet")]
        if not parquet_files:
            print("⚠️ No .parquet files found in that folder.")
            return None
        print("\nAvailable .parquet files:")
        for i, obj in enumerate(parquet_files):
            print(f"[{i}] {obj.object_name}")
        idx = int(input("\nSelect file index to fetch: ").strip())
        selected_obj = parquet_files[idx]
        print(f"\nFetching {selected_obj.object_name}")
        response = minio_client.get_object(bucket_name, selected_obj.object_name)
        
        # Read Parquet data into a DataFrame
        data = pd.read_parquet(BytesIO(response.read()), engine="pyarrow")
        
        # Display sample of data (but return the DataFrame, not JSON)
        print(f"Successfully fetched data from {selected_obj.object_name}")
        print(f"Sample data (first few rows):")
        print(data.head())
        
        # Return the DataFrame directly
        return data
    except Exception as e:
        print(f"❌ Error: {e}")
        return None

def fetch_and_split_parquet_from_minio(bucket_name="wl-data", max_workers=8):
    """
    Prompts for a MinIO folder, lists all .parquet files,
    then randomly splits them into three groups:
      - 70% for training
      - 20% for validation
      - 10% for testing
    Fetches and processes the files in parallel in memory.
    Returns a dictionary of DataFrames for each group.
    """
    load_dotenv()
    minio_client = Minio(
        os.getenv("MINIO_ENDPOINT"),
        access_key=os.getenv("MINIO_ACCESS_KEY"),
        secret_key=os.getenv("MINIO_SECRET_KEY"),
        secure=os.getenv("MINIO_SECURE").lower() == 'true',
    )
    folder = input("Enter MinIO folder name: ").strip()
    prefix = f"{folder}/"

    # list remote parquet files
    objects = list(minio_client.list_objects(bucket_name, prefix=prefix, recursive=True))
    parquet_files = [obj for obj in objects if obj.object_name.endswith(".parquet")]

    total = len(parquet_files)
    if total == 0:
        print("⚠️ No .parquet files found in that folder.")
        return None

    print(f"\nFound {total} .parquet files in '{folder}'.")
    n = int(input(f"How many files would you like to process? (1–{total}): ").strip())
    n = max(1, min(n, total))

    # shuffle and take only the first n
    random.shuffle(parquet_files)
    selected = parquet_files[:n]
    print(f"\n→ Processing {n} files:\n" +
          "\n".join(f"  • {obj.object_name}" for obj in selected))

    # split counts
    train_count = int(0.7 * n)
    val_count = int(0.2 * n)
    test_count = n - train_count - val_count
    print(f"\nSplit into: Train={train_count}, Val={val_count}, Test={test_count}")

    # assign slices
    train_files = selected[:train_count]
    val_files   = selected[train_count: train_count + val_count]
    test_files  = selected[train_count + val_count:]

    # fetch in parallel
    print("\nFetching training files...")
    train_data = fetch_files_parallel(train_files, minio_client, bucket_name, max_workers)
    print("Fetching validation files...")
    val_data = fetch_files_parallel(val_files, minio_client, bucket_name, max_workers)
    print("Fetching test files...")
    test_data = fetch_files_parallel(test_files, minio_client, bucket_name, max_workers)

    # Display sample of fetched data
    if train_data:
        print("\nSample of training data:")
        print(train_data[0].head())
    
    print("\n✅ Done.")
    return {"train": train_data, "val": val_data, "test": test_data}

def display_dataframes(data_dict):
    """
    Display the fetched dataframes
    """
    for key, dataframes in data_dict.items():
        print(f"\n=== {key.upper()} DATA ===")
        for i, df in enumerate(dataframes):
            print(f"\nDataset {i+1} (first 5 rows):")
            print(df.head())

if __name__ == "__main__":
    option = input("Choose an option:\n1. Fetch a single parquet file\n2. Fetch and split multiple files\nEnter choice (1-2): ")
    
    if option == "1":
        fetch_selected_parquet_from_minio()
    elif option == "2":
        result = fetch_and_split_parquet_from_minio()
        display_option = input("\nWould you like to display all fetched data? (y/n): ").lower().strip()
        if display_option == 'y':
            display_dataframes(result)
    else:
        print("Invalid option selected.")