#!/usr/bin/env python3
import sys
import time
import os
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from mz_schemas_protobuf.Pose_pb2 import Pose
import json
import requests

# --- Configuration ---
FLASK_URL = "http://localhost:5000/position"
ROUTE_ID = None  # Will be set from server or manually

# Optional logging
session_name = "live_gnss"
log_dir = f"logs/{session_name}"
os.makedirs(log_dir, exist_ok=True)
log_file_path = os.path.join(log_dir, "subscriber_log.txt")
log_f = open(log_file_path, "w")
sys.stdout = log_f

def get_active_route_id():
    """Fetch the currently active route from Flask server."""
    try:
        resp = requests.get("http://localhost:5000/health", timeout=0.5)
        if resp.ok:
            return getattr(resp.json(), 'active_route_id', None)
    except:
        return None

# --- GNSS Callback ---
def gnss_callback(topic_name, pose: Pose, timestamp):
    try:
        lat = pose.lat_lon_ht.latitude_deg
        lon = pose.lat_lon_ht.longitude_deg
        yaw = pose.orientation.yaw_rad

        print(f"Timestamp: {timestamp} | Lat: {lat:.6f} | Lon: {lon:.6f} | Yaw: {yaw:.3f}")

        payload = {"lat": lat, "lon": lon}
        requests.post("http://localhost:5000/update_gnss", json=payload)

    except Exception as e:
        print("Error sending GNSS data:", e)

# --- Main ---
if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "Live GNSS Bridge")

    sub = ProtoSubscriber("rec_gnss", Pose)
    sub.set_callback(gnss_callback)

    print("Live GNSS bridge started... listening on 'rec_gnss'")

    try:
        while ecal_core.ok():
            time.sleep(0.02)  # 50 Hz
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        sys.stdout.close()
        sys.stdout = sys.__stdout__
        ecal_core.finalize()
