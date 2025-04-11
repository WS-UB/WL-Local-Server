import os
import random
from concurrent.futures import ThreadPoolExecutor
from minio import Minio
from io import BytesIO
from dotenv import load_dotenv

def download_file_parallel(minio_client, bucket_name, obj, target_dir):
    """
    Download a single object from MinIO and save it to target_dir.
    """
    try:
        print(f"📥 Downloading {obj.object_name}")
        response = minio_client.get_object(bucket_name, obj.object_name)
        safe_filename = os.path.basename(obj.object_name)
        local_filename = os.path.join(target_dir, safe_filename)
        with open(local_filename, "wb") as f:
            # Download the whole file at once for faster performance
            f.write(response.read())
        print(f"✅ Saved to {local_filename}")
        return local_filename
    except Exception as e:
        print(f"❌ Failed to download {obj.object_name}: {e}")
        return None

def download_files_parallel(file_list, target_dir, minio_client, bucket_name, max_workers=8):
    """
    Download a list of files in parallel using ThreadPoolExecutor.
    Returns a list of local file paths.
    """
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        results = list(executor.map(
            lambda obj: download_file_parallel(minio_client, bucket_name, obj, target_dir), file_list))
    local_paths = [path for path in results if path is not None]
    return local_paths

def fetch_selected_parquet_from_minio(bucket_name="wl-data"):
    """
    Prompts for a MinIO folder and lists all .parquet files.
    Allows the user to choose one file to download into the local "data" folder,
    and then returns the local file path.
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
    os.makedirs("data", exist_ok=True)
    try:
        objects = list(minio_client.list_objects(bucket_name, prefix=prefix, recursive=True))
        parquet_files = [obj for obj in objects if obj.object_name.endswith(".parquet")]
        if not parquet_files:
            print("⚠️ No .parquet files found in that folder.")
            return None
        print("\n📄 Available .parquet files:")
        for i, obj in enumerate(parquet_files):
            print(f"[{i}] {obj.object_name}")
        idx = int(input("\nSelect file index to download: ").strip())
        selected_obj = parquet_files[idx]
        print(f"\n📥 Downloading {selected_obj.object_name}")
        response = minio_client.get_object(bucket_name, selected_obj.object_name)
        local_filename = os.path.join("data", os.path.basename(selected_obj.object_name))
        with open(local_filename, "wb") as f:
            f.write(response.read())
        print(f"✅ Saved to {local_filename}")
        return local_filename
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
    Downloads the files in parallel into local subdirectories:
      - data/train_data_path
      - data/val_data_path
      - data/test_data_path
    Returns a dictionary of local file paths for each group.
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
    base_dir = "data"
    train_dir = os.path.join(base_dir, "train_data_path")
    val_dir = os.path.join(base_dir, "val_data_path")
    test_dir = os.path.join(base_dir, "test_data_path")
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(val_dir, exist_ok=True)
    os.makedirs(test_dir, exist_ok=True)
    try:
        objects = list(minio_client.list_objects(bucket_name, prefix=prefix, recursive=True))
        parquet_files = [obj for obj in objects if obj.object_name.endswith(".parquet")]
        if not parquet_files:
            print("⚠️ No .parquet files found in that folder.")
            return None
        random.shuffle(parquet_files)
        total = len(parquet_files)
        train_count = int(0.7 * total)
        val_count = int(0.2 * total)
        test_count = total - train_count - val_count
        print(f"\nTotal files: {total} | Train: {train_count} | Val: {val_count} | Test: {test_count}")
        train_files = parquet_files[:train_count]
        val_files = parquet_files[train_count: train_count + val_count]
        test_files = parquet_files[train_count + val_count:]
        print("\nStarting parallel download for training files...")
        train_paths = download_files_parallel(train_files, train_dir, minio_client, bucket_name, max_workers)
        print("Training files download complete.\n")
        print("Starting parallel download for validation files...")
        val_paths = download_files_parallel(val_files, val_dir, minio_client, bucket_name, max_workers)
        print("Validation files download complete.\n")
        print("Starting parallel download for test files...")
        test_paths = download_files_parallel(test_files, test_dir, minio_client, bucket_name, max_workers)
        print("Test files download complete.\n")
        return {"train": train_paths, "val": val_paths, "test": test_paths}
    except Exception as e:
        print(f"❌ Error: {e}")
        return None
