# ==============================================================================
# === 1. IMPORT PUSTAKA ===
# ==============================================================================
import cv2
import time
import math
import threading
import io
import os
import numpy as np
from datetime import datetime
from PIL import Image

# Library AI & Hardware
from ultralytics import YOLO
from pymavlink import mavutil

# Library Cloud & Database
import cloudinary
import cloudinary.uploader
import firebase_admin
from firebase_admin import credentials, db

# ==============================================================================
# === 2. KONFIGURASI GLOBAL ===
# ==============================================================================

# --- A. Konfigurasi Cloudinary ---
cloudinary.config(
    cloud_name = "ds2j86wki",        
    api_key    = "563889432822331",   
    api_secret = "Nr7nKUADbjW3w2E-SwCt1EBfC6E" 
)

# --- B. Konfigurasi Firebase (Realtime Database) ---
FIREBASE_KEY_PATH = 'firebase-key.json' 
FIREBASE_URL      = 'https://asvmonitoringweb-default-rtdb.asia-southeast1.firebasedatabase.app/'
try:
    if not firebase_admin._apps:
        cred = credentials.Certificate(FIREBASE_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': FIREBASE_URL})
    ref = db.reference('/kapal/tim-asv-01')
    print("✅ Firebase & Cloudinary Terhubung.")
except Exception as e:
    print(f"❌ Gagal koneksi Firebase: {e}")
    ref = None

# --- C. Konfigurasi Kamera & Model (2 KAMERA) ---
CAM_INDEX_NAV_SURFACE = 0  # Kamera 1: Navigasi YOLO sekaligus Surface Imaging
CAM_INDEX_UNDER       = 1  # Kamera 2: Khusus Underwater Imaging

MODEL_PATH = 'best.onnx' # Menggunakan ONNX agar ringan

# Konfigurasi Deteksi & Misi
CONFIDENCE_THRESHOLD = 0.3      # Threshold untuk bola navigasi
MISSION_CONF_THRESHOLD = 0.6    # Threshold LEBIH KETAT untuk foto misi (Anti-salah trigger)
CAPTURE_COOLDOWN     = 5.0      # Jeda waktu antar foto (detik)
REQUIRED_FRAMES      = 5        # Objek misi harus terdeteksi 5 frame berturut-turut

# Variabel Global untuk Anti-Salah Trigger
detection_counter = {"surface": 0, "underwater": 0}

# --- D. Konfigurasi MAVLink ---
MAVLINK_CONNECTION_STR = '/dev/ttyACM0' 
BAUDRATE = 115200

# --- E. Konfigurasi Navigasi (GPS & VISUAL) ---
TARGET_WAYPOINT = {
    'lat': -7.052600,
    'lon': 110.434800
}

CRUISE_SPEED = 1.0       
APPROACH_SPEED = 0.8     
VISUAL_P_GAIN = 0.002    
GPS_P_GAIN    = 0.05     

# Mapping Class YOLO (Sesuaikan dengan file data.yaml)
CLASS_NAMES_VIS = {0: 'green_ball', 1: 'red_ball', 2: 'target_surface', 3: 'target_underwater'}
CLASS_COLORS = [(0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
CLASS_IDS_QUERY = {'green_ball': 0, 'red_ball': 1, 'target_surface': 2, 'target_underwater': 3}

FRAME_WIDTH, FRAME_HEIGHT = 640, 640

# ==============================================================================
# === 3. STRUKTUR DATA ===
# ==============================================================================

shared_data = {
    "telemetry": None,       
    "capture_trigger": None, 
    "capture_frame": None    
}
data_lock = threading.Lock()

data_payload = {
    "position_log": {
        "preparation": "In Progress",
        "start": "Pending",
        "floating_ball": 0,
        "surface_imaging": "Pending",
        "underwater_imaging": "Pending",
        "finish": "Pending"
    },
    "attitude": {"sog": 0.0, "cog": 0.0, "heading": 0.0},
    "local_position": {"x": 0.0, "y": 0.0}, 
    "gps_location": {"lat": 0.0, "lon": 0.0},
    "current_mission": "Autonomous",
    "mission_images": {"surface": None, "underwater": None},
    "track_id": "A",                
    "race_start_timestamp": None,   
    "race_finish_timestamp": None,  
    "indicators": {"battery": 100, "last_update": None}
}

# ==============================================================================
# === 4. FUNGSI HELPER ===
# ==============================================================================

def calculate_tracking_logic(results, frame_width, frame_height):
    global detection_counter
    frame_center_x = frame_width // 2
    
    # [SOLUSI 1] Batas Grid Fungsional (Region of Interest)
    # Abaikan semua objek di 1/3 bagian atas layar (horizon/daratan jauh)
    horizon_y_limit = frame_height // 3 
    
    detected_objects = {}
    auto_trigger = None
    
    # Flag untuk mengecek target misi
    surface_in_frame = False
    underwater_in_frame = False

    # [SOLUSI 2] List sementara untuk menampung BANYAK bola sebelum disaring
    red_balls_list = []
    green_balls_list = []

    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            box_area = (x2 - x1) * (y2 - y1) # Hitung luas kotak untuk mencari jarak terdekat

            # Abaikan bola yang berada di atas garis horizon (Terlalu jauh)
            if center_y < horizon_y_limit:
                continue 
            
            # Kumpulkan semua bola yang ada di area valid
            if cls_id == 0: 
                green_balls_list.append({'x': center_x, 'y': center_y, 'area': box_area})
            elif cls_id == 1: 
                red_balls_list.append({'x': center_x, 'y': center_y, 'area': box_area})
            
            # --- LOGIKA ANTI-SALAH TRIGGER (MISI) ---
            elif cls_id == 2 and conf > MISSION_CONF_THRESHOLD:
                surface_in_frame = True
                detection_counter["surface"] += 1
                if detection_counter["surface"] >= REQUIRED_FRAMES:
                    auto_trigger = 'surface'
                    
            elif cls_id == 3 and conf > MISSION_CONF_THRESHOLD:
                underwater_in_frame = True
                detection_counter["underwater"] += 1
                if detection_counter["underwater"] >= REQUIRED_FRAMES:
                    auto_trigger = 'underwater'

    # Reset counter jika objek tiba-tiba hilang
    if not surface_in_frame: detection_counter["surface"] = 0
    if not underwater_in_frame: detection_counter["underwater"] = 0

    # ==========================================
    # FILTERING: Pilih Pasangan Bola Terdekat
    # ==========================================
    if red_balls_list:
        # Urutkan bola merah dari yang TERBESAR (terdekat) ke terkecil
        red_balls_list.sort(key=lambda b: b['area'], reverse=True)
        best_red = red_balls_list[0] # Ambil yang index 0
        detected_objects['red_ball'] = (best_red['x'], best_red['y'])

    if green_balls_list:
        # Urutkan bola hijau dari yang TERBESAR (terdekat) ke terkecil
        green_balls_list.sort(key=lambda b: b['area'], reverse=True)
        best_green = green_balls_list[0]
        detected_objects['green_ball'] = (best_green['x'], best_green['y'])

    # Kalkulasi Navigasi Visual (Bola Merah & Hijau Terdekat)
    if 'red_ball' in detected_objects and 'green_ball' in detected_objects:
        rx, ry = detected_objects['red_ball']
        gx, gy = detected_objects['green_ball']
        midpoint_x = (rx + gx) / 2
        midpoint_y = (ry + gy) / 2
        offset_value = int(midpoint_x - frame_center_x)

        return {
            "status": "TRACKING",
            "offset": offset_value,
            "auto_trigger": auto_trigger,
            "coords": {
                "red": (int(rx), int(ry)),
                "green": (int(gx), int(gy)),
                "midpoint": (int(midpoint_x), int(midpoint_y))
            }
        }
    else:
        return {"status": "GPS_NAV", "offset": 0, "auto_trigger": auto_trigger, "coords": detected_objects}

# ... (Fungsi matematika, MAVLink, Firebase, dan Cloudinary) ...
def get_bearing_to_target(current_lat, current_lon, target_lat, target_lon):
    dLon = math.radians(target_lon - current_lon)
    lat1 = math.radians(current_lat)
    lat2 = math.radians(target_lat)
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.atan2(y, x)
    return math.degrees(bearing) % 360

def set_mode(conn, mode_name):
    if conn is None: return
    try:
        mode_id = conn.mode_mapping()[mode_name]
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"✅ Mode diganti ke: {mode_name}")
    except Exception as e: pass

def send_ned_velocity(conn, velocity_x, velocity_y, yaw_rate):
    if conn is None: return
    type_mask = 0b0000011111000111 
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, type_mask,
        0, 0, 0, velocity_x, velocity_y, 0, 0, 0, 0, 0, yaw_rate
    )

def archive_and_reset_data():
    if ref is None: return
    try:
        old_data = ref.get()
        if old_data:
            timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            db.reference(f'/arsip/lomba_{timestamp_str}').set(old_data)
            ref.set(data_payload)
    except Exception: pass

def open_mavlink():
    print(f"Menghubungkan MAVLink ke {MAVLINK_CONNECTION_STR}...")
    try:
        conn = mavutil.mavlink_connection(MAVLINK_CONNECTION_STR, baud=BAUDRATE)
        conn.wait_heartbeat(timeout=3)
        print("✅ Connected to Vehicle!")
        return conn
    except:
        print("⚠️ Gagal connect MAVLink (Mode Simulasi).")
        return None

def get_vehicle_data(connection):
    if connection is None: return None
    try:
        msg_gps = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        msg_bat = connection.recv_match(type='SYS_STATUS', blocking=False)
        msg_vfr = connection.recv_match(type='VFR_HUD', blocking=False)
        if not all([msg_gps, msg_vfr, msg_bat]): return None
        return {
            "gps": {'lat': msg_gps.lat / 1e7, 'lon': msg_gps.lon / 1e7, 'cog': msg_gps.hdg / 100.0},
            "bat_status": {'level': msg_bat.battery_remaining},
            "heading": msg_vfr.heading,
            "speed": {'ground_speed': msg_vfr.groundspeed}
        }
    except: return None

def capture_clean_frame(cap_device):
    if not cap_device or not cap_device.isOpened(): return None
    for _ in range(3): cap_device.grab() 
    ret, frame = cap_device.read()
    return frame if ret else None

def draw_geotag_on_image(frame, sensor_data):
    if sensor_data is None: sensor_data = {}
    lat = sensor_data.get("gps", {}).get("lat", 0.0)
    lon = sensor_data.get("gps", {}).get("lon", 0.0)
    cv2.putText(frame, f"GPS: {lat:.5f}, {lon:.5f}", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    return frame

def upload_to_cloudinary(frame):
    try:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        buffer = io.BytesIO()
        img.save(buffer, format="JPEG", quality=80)
        buffer.seek(0)
        res = cloudinary.uploader.upload(buffer, folder="asv_lomba")
        return res.get('secure_url')
    except: return None

# ==============================================================================
# === 5. WORKER THREAD ===
# ==============================================================================
def worker_thread_task():
    global data_payload
    while True:
        with data_lock:
            telemetry = shared_data["telemetry"]
            trigger = shared_data["capture_trigger"]
            frame_cap = shared_data["capture_frame"]
            if trigger:
                shared_data["capture_trigger"] = None
                shared_data["capture_frame"] = None

        if ref is None: 
            time.sleep(1)
            continue

        if telemetry:
            try:
                data_payload["gps_location"]["lat"] = telemetry["gps"]['lat']
                data_payload["gps_location"]["lon"] = telemetry["gps"]['lon']
                data_payload["attitude"]["heading"] = telemetry["heading"]
            except: pass

        if trigger and frame_cap is not None:
            status_key = f"{trigger}_imaging" 
            data_payload["position_log"][status_key] = "In Progress"
            final_img = draw_geotag_on_image(frame_cap.copy(), telemetry)
            url = upload_to_cloudinary(final_img)
            if url:
                data_payload["mission_images"][trigger] = url
                data_payload["position_log"][status_key] = "Done"
            else:
                data_payload["position_log"][status_key] = "Failed"

        try:
            data_payload["indicators"]["last_update"] = time.time()
            ref.set(data_payload)
        except: pass
        time.sleep(0.5)

# ==============================================================================
# === 6. MAIN LOOP (HYBRID SYSTEM 2 KAMERA) ===
# ==============================================================================
def main_system():
    conn = open_mavlink()

    if conn:
        print("Mengatur mode ke GUIDED...")
        set_mode(conn, 'GUIDED')
        time.sleep(1) 
    
    print("Membuka 2 Kamera...")
    cap_main  = cv2.VideoCapture(CAM_INDEX_NAV_SURFACE) # Kamera Navigasi & Surface
    cap_under = cv2.VideoCapture(CAM_INDEX_UNDER)       # Kamera Underwater

    if not cap_main.isOpened(): 
        print("❌ Kamera Utama gagal terbuka!")
        return

    print(f"Memuat Model YOLO ONNX: {MODEL_PATH} ...")
    try: 
        model = YOLO(MODEL_PATH, task='detect') 
    except Exception as e: 
        print(f"❌ Gagal memuat model: {e}")
        return

    last_capture_time = 0
    manual_trigger_request = None 

    print("\n✅ SYSTEM RUNNING: 2 CAMERA HYBRID (AUTO + MANUAL OVERRIDE)")
    
    try:
        while True:
            # A. Baca Frame dari Kamera Utama
            ret, frame = cap_main.read()
            if not ret: break
            frame_resized = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            # B. Telemetri
            telemetry = get_vehicle_data(conn)
            
            # C. Deteksi YOLO
            results = model.predict(frame_resized, conf=CONFIDENCE_THRESHOLD, verbose=False, max_det=5)
            
            # D. LOGIC DETEKSI (Sudah menggunakan FRAME_HEIGHT untuk ROI)
            logic_data = calculate_tracking_logic(results, FRAME_WIDTH, FRAME_HEIGHT)
            status = logic_data["status"]
            offset = logic_data["offset"]
            coords = logic_data.get("coords", {})
            auto_trigger = logic_data["auto_trigger"] 
            
            # Visualisasi Bounding Box
            for r in results:
                for box in r.boxes:
                     x1, y1, x2, y2 = box.xyxy[0].tolist()
                     cls = int(box.cls[0].item())
                     conf = float(box.conf[0])
                     color = CLASS_COLORS[cls] if cls < len(CLASS_COLORS) else (255,255,255)
                     
                     # Gambar box dan confidence
                     cv2.rectangle(frame_resized, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                     cv2.putText(frame_resized, f"{CLASS_NAMES_VIS[cls]} {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # E. CAPTURE LOGIC (HYBRID: AUTO ATAU MANUAL)
            now = time.time()
            
            # Tentukan apakah ada trigger yang valid (Manual memprioritaskan Auto)
            final_trigger = manual_trigger_request if manual_trigger_request else auto_trigger

            if final_trigger and (now - last_capture_time) > CAPTURE_COOLDOWN:
                target_cam = cap_main if final_trigger == 'surface' else cap_under
                
                if target_cam and target_cam.isOpened():
                    snap = capture_clean_frame(target_cam)
                    if snap is not None:
                        with data_lock:
                            shared_data["capture_trigger"] = final_trigger
                            shared_data["capture_frame"] = snap
                        last_capture_time = now
                        
                        trigger_source = "MANUAL" if manual_trigger_request else "AI AUTO"
                        print(f"📸 {trigger_source} CAPTURE: {final_trigger.upper()}")
                        cv2.putText(frame_resized, f"{trigger_source} SNAP: {final_trigger.upper()}", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,255), 3)
                
                # Reset request manual setelah diproses
                manual_trigger_request = None

            # [TAMBAHAN] GAMBAR GRID & HORIZON (IGNORE ZONE)
            h, w = FRAME_HEIGHT, FRAME_WIDTH
            cv2.line(frame_resized, (w // 4, 0), (w // 4, h), (255, 255, 255), 1)
            cv2.line(frame_resized, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
            cv2.line(frame_resized, (3 * w // 4, 0), (3 * w // 4, h), (255, 255, 255), 1)
            cv2.line(frame_resized, (0, h // 2), (w, h // 2), (255, 255, 255), 1)
            
            horizon_y = h // 3
            cv2.line(frame_resized, (0, horizon_y), (w, horizon_y), (0, 0, 255), 2)
            cv2.putText(frame_resized, "IGNORE ZONE (NO DETECT)", (10, horizon_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # F. KONTROL HYBRID NAVIGASI
            vx = 0.0  
            yaw_rate = 0.0 

            if status == "TRACKING":
                yaw_rate = offset * VISUAL_P_GAIN 
                yaw_rate = max(min(yaw_rate, 0.5), -0.5)
                vx = APPROACH_SPEED 
                
                if "midpoint" in coords:
                    mx = coords['midpoint']
                    # Gambar titik tengah
                    cv2.circle(frame_resized, mx, 8, (0, 255, 255), -1)
                    
                    # [TAMBAHAN] GAMBAR GATE (Garis antar bola)
                    if 'red' in coords and 'green' in coords:
                        cv2.line(frame_resized, coords['green'], coords['red'], (255, 255, 0), 2)
                        cv2.putText(frame_resized, "GATE", (mx[0] + 15, mx[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                    # [TAMBAHAN] GAMBAR VIRTUAL LINE
                    posisi_kapal = (w // 2, h)
                    cv2.line(frame_resized, posisi_kapal, mx, (255, 0, 255), 3)
                    cv2.putText(frame_resized, "VIRTUAL LINE", (posisi_kapal[0] + 10, posisi_kapal[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                cv2.putText(frame_resized, f"VISUAL LOCK: {offset}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            else:

                if telemetry:
                    curr_lat = telemetry['gps']['lat']
                    curr_lon = telemetry['gps']['lon']
                    curr_heading = telemetry['heading']
                    target_bearing = get_bearing_to_target(curr_lat, curr_lon, TARGET_WAYPOINT['lat'], TARGET_WAYPOINT['lon'])
                    heading_error = target_bearing - curr_heading
                    if heading_error > 180: heading_error -= 360
                    if heading_error < -180: heading_error += 360
                    yaw_rate = heading_error * GPS_P_GAIN
                    yaw_rate = max(min(yaw_rate, 0.5), -0.5) 
                    vx = CRUISE_SPEED 
                    cv2.putText(frame_resized, f"GPS NAV -> Target: {target_bearing:.1f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,255), 2)
                else:
                    cv2.putText(frame_resized, "NO GPS & NO VISUAL", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            # G. KIRIM PERINTAH KE MOTOR
            send_ned_velocity(conn, vx, 0, yaw_rate)

            # H. Update Thread & Display
            with data_lock: shared_data["telemetry"] = telemetry
            
            # Status Deteksi Anti-Salah di Layar
            cv2.putText(frame_resized, f"AutoFilter -> S: {detection_counter['surface']}/{REQUIRED_FRAMES} | U: {detection_counter['underwater']}/{REQUIRED_FRAMES}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
            cv2.putText(frame_resized, "[S]: Surface | [U]: Under | [Q]: Quit", (10, FRAME_HEIGHT - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
            
            cv2.imshow("ASV Hybrid Control", frame_resized)
            
            # I. KEYBOARD LISTENER (MANUAL OVERRIDE)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('s'): manual_trigger_request = 'surface'
            elif key == ord('u'): manual_trigger_request = 'underwater'

    finally:
        send_ned_velocity(conn, 0, 0, 0)
        print("Menutup sistem...")
        cap_main.release()
        cap_under.release()
        if conn: conn.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    archive_and_reset_data()
    t = threading.Thread(target=worker_thread_task, daemon=True)
    t.start()
    main_system()