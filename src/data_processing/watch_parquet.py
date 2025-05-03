import os
import sys
import time
from io import BytesIO
import pandas as pd
from minio import Minio
from dotenv import load_dotenv

# ── 1) Compute PROJECT_ROOT (two levels up: data_processing -> src -> project root) ──
PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir, os.pardir)
)
dotenv_path = os.path.join(PROJECT_ROOT, ".env")
if not os.path.exists(dotenv_path):
    print(f"⚠️  Warning: .env not found at {dotenv_path}")
# Load the real credentials from project‐root .env
load_dotenv(dotenv_path, override=True)

# ── 2) Read MinIO settings from env (no more defaults unless truly missing) ──
MINIO_ENDPOINT = os.getenv("MINIO_ENDPOINT")
MINIO_SECURE   = os.getenv("MINIO_SECURE", "false").lower() == "true"
MINIO_ACCESS   = os.getenv("MINIO_ACCESS_KEY")
MINIO_SECRET   = os.getenv("MINIO_SECRET_KEY")
MINIO_BUCKET   = os.getenv("MINIO_BUCKET", "wl-data")

print("Using MinIO endpoint:", MINIO_ENDPOINT)
print("Using MINIO_SECURE   =", MINIO_SECURE)
print("Using MINIO_ACCESS_KEY=", MINIO_ACCESS)

# ── 3) Add your src/ folder to PYTHONPATH, then import the processor ──
SRC_PATH = os.path.join(PROJECT_ROOT, "src")
sys.path.insert(0, SRC_PATH)
from data_processing.csi_calibration import process_parquet

# ── 4) Initialize MinIO client ──
minio_client = Minio(
    MINIO_ENDPOINT,
    access_key=MINIO_ACCESS,
    secret_key=MINIO_SECRET,
    secure=MINIO_SECURE,
)

# ── 5) Helpers to list user‐folders and get their latest parquet ──
def list_user_ids(bucket=MINIO_BUCKET):
    return {
        obj.object_name.split("/", 1)[0]
        for obj in minio_client.list_objects(bucket, "", recursive=True)
    }

def latest_parquet_for(uid, bucket=MINIO_BUCKET):
    files = [
        o for o in minio_client.list_objects(bucket, f"{uid}/", recursive=True)
        if o.object_name.endswith(".parquet")
    ]
    if not files:
        return None
    return max(files, key=lambda o: o.last_modified).object_name

# ── 6) Watcher loop: process each new file exactly once ──
def main(poll_interval: float = 5.0):
    seen = {
        obj.object_name
        for obj in minio_client.list_objects(MINIO_BUCKET, recursive=True)
        if obj.object_name.endswith(".parquet")
    }

    print("👀 Watching for NEW .parquet files in MinIO…")

    while True:
        for uid in list_user_ids():
            key = latest_parquet_for(uid)
            if key and key not in seen:
                print(f"[+] Detected new file: {key} (user={uid})")
                try:
                    process_parquet(key, uid)  # no `bucket` kwarg anymore
                    print(f"    → Successfully processed {key}")
                except Exception as e:
                    print(f"    ! Error processing {key}:", e)
                seen.add(key)
        time.sleep(poll_interval)

if __name__ == "__main__":
    main()