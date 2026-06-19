#!/usr/bin/env python3
"""Cek koneksi hardware: kamera, LiDAR, serial motor, GPS."""

import cv2
import os
import glob
import serial
import sys
import time


def cek_kamera(dev):
    path = f"/dev/video{dev}" if isinstance(dev, int) else dev
    cap = cv2.VideoCapture(int(dev) if isinstance(dev, int) or dev.isdigit() else dev)
    ok = cap.isOpened()
    if ok:
        w, h = int(cap.get(3)), int(cap.get(4))
        fps = cap.get(5)
        print(f"  ✅  Kamera {dev} — {w}x{h} @ {fps:.0f}fps")
        cap.release()
    else:
        print(f"  ❌  Kamera {dev} — tidak terdeteksi")
    return ok


def cek_serial(path, baud=115200):
    try:
        s = serial.Serial(path, baud, timeout=0.5)
        s.close()
        print(f"  ✅  Serial {path} @ {baud} — terbuka")
        return True
    except Exception as e:
        print(f"  ❌  Serial {path} — {e}")
        return False


def cek_rplidar(path):
    try:
        from rplidar import RPLidar
        lidar = RPLidar(path)
        info = lidar.get_info()
        lidar.stop()
        lidar.disconnect()
        print(f"  ✅  RPLidar {path} — {info}")
        return True
    except Exception as e:
        print(f"  ❌  RPLidar {path} — {e}")
        return False


def detect_gps(port):
    """Deteksi apakah port serial ngirim data GPS (NMEA)."""
    try:
        s = serial.Serial(port, 9600, timeout=1)
        data = s.readline()
        s.close()
        if data and data.startswith(b"$G"):
            print(f"  ✅  GPS {port} — NMEA detected: {data[:30].decode(errors='ignore')}")
            return True
        else:
            print(f"  ⚠️   Serial {port} — bukan GPS (data: {data[:20]})")
            return False
    except:
        return False


def main():
    print("=" * 50)
    print("  CEK HARDWARE — ASV1")
    print("=" * 50)

    # --- TTY ports ---
    print("\n📡  Serial Ports (/dev/tty*)")
    ttys = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") +
                  glob.glob("/dev/ttyS*") + glob.glob("/dev/ttyAMA*"))
    if not ttys:
        print("  ⚠️   Tidak ada port serial terdeteksi")
    else:
        for tty in ttys:
            print(f"  🔌  {tty}")
            cek_serial(tty)
            time.sleep(0.1)

    # --- RPLidar ---
    print("\n🛞   LiDAR")
    lidar_ports = [p for p in ttys if "USB" in p or "ACM" in p]
    found_lidar = False
    for p in lidar_ports:
        if cek_rplidar(p):
            found_lidar = True
    if not found_lidar:
        print("  ⚠️   RPLidar tidak terdeteksi (colok USB / izin port)")

    # --- Kamera ---
    print("\n📷   Kamera")
    v4l_devs = sorted(glob.glob("/dev/video*"))
    video_devs = [d for d in v4l_devs if not d.endswith("meta")]
    found_cam = False
    for d in video_devs:
        if "/dev/video" in d:
            dev_num = d.replace("/dev/video", "")
            if cek_kamera(dev_num):
                found_cam = True
    if not found_cam:
        print("  ❌  Tidak ada kamera terdeteksi")

    # --- Network check ---
    print("\n🌐   Network")
    import subprocess
    r = subprocess.run(["ping", "-c", "1", "-W", "1", "8.8.8.8"],
                       capture_output=True, text=True)
    if r.returncode == 0:
        print("  ✅  Internet OK")
    else:
        print("  ⚠️   Internet tidak reachable (cek WiFi/Ethernet)")

    print("\n" + "=" * 50)
    print("  Done.")
    print("=" * 50)


if __name__ == "__main__":
    main()
