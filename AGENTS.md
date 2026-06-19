You are a ROS2 robotics AI assistant for a low-resource system (8GB RAM laptop).

Your main goal is to help build a full robotics stack using:
- ROS2 (rclpy and rclcpp)
- LiDAR (LaserScan)
- Camera (OpenCV + YOLO)
- Basic autonomous navigation

You must always follow this workflow:

1. PLANNING STAGE
- Break problem into ROS2 nodes
- Define inputs/outputs for each node
- Keep architecture simple and modular

2. CODING STAGE
- Write clean Python (rclpy) or C++ (rclcpp)
- Prefer Python for AI / vision tasks
- Prefer C++ only for performance-critical nodes
- Code must be minimal, runnable, and ROS2-compliant

3. DEBUGGING STAGE
- Analyze ROS2 errors (colcon, runtime, topics)
- Provide exact terminal commands to fix issues
- Never guess blindly, always explain cause

HARD CONSTRAINTS:
- Optimize for 8GB RAM system
- Avoid heavy models unless explicitly requested
- Keep memory usage low
- Prefer lightweight YOLO models (yolov8n)

PROJECT FOCUS:
- LiDAR obstacle detection
- Camera object detection
- Sensor fusion (LiDAR + vision)
- Simple autonomous robot logic

## SESSION LOG — 17 June 2026

### ✅ Done — `asv1` Package (Modular ASV dari `program.py`)

### Architecture (asv1)
| Node | Bahasa | File | Fungsi |
|------|--------|------|--------|
| `node_kamera` | Python | `asv1/node_kamera.py` | Baca 3 kamera (utama/bawah/samping), pub `/asv/kamera/*` |
| `node_deteksi` | Python | `asv1/node_deteksi.py` | YOLO + tracking logic (calculate_tracking_logic), pub `/asv/tracking`, `/asv/deteksi`, `/asv/trigger`, `/asv/vizu` |
| `node_lidar` | Python | `asv1/node_lidar.py` | RPLidar C1 → 4 sektor obstacle `/asv/obstacle` + `/scan` |
| `node_navigasi` | C++ | `src/node_navigasi.cpp` | Hybrid control: **Obstacle > Visual > GPS**, pub `/asv/cmd_vel` |
| `node_motor` | C++ | `src/node_motor.cpp` | Serial MAVLink/MSP/raw, pub `/asv/telemetri` |
| `node_misi` | Python | `asv1/node_misi.py` | Firebase + Cloudinary + capture trigger |

### Topics (asv1)
| Topic | Type | Pub | Sub |
|-------|------|-----|-----|
| `/asv/kamera/utama` | Image | node_kamera | node_deteksi, node_misi |
| `/asv/kamera/bawah` | Image | node_kamera | node_deteksi, node_misi |
| `/asv/kamera/samping` | Image | node_kamera | node_deteksi, node_misi |
| `/asv/deteksi` | Float32MultiArray | node_deteksi | node_misi |
| `/asv/tracking` | Float32MultiArray | node_deteksi | node_navigasi |
| `/asv/trigger` | Int32 | node_deteksi | node_misi |
| `/asv/vizu` | Image | node_deteksi | viewer |
| `/asv/obstacle` | Float32MultiArray | node_lidar | node_navigasi |
| `/asv/cmd_vel` | Twist | node_navigasi | node_motor |
| `/asv/telemetri` | Float32MultiArray | node_motor | node_navigasi, node_misi |
| `/scan` | LaserScan | node_lidar | (optional) |

### Priority Control (node_navigasi)
1. **Obstacle avoidance** — stop/turn if obstacle < threshold
2. **Visual tracking** — steer toward midpoint of red+green balls
3. **GPS navigation** — bearing to waypoint (fallback)

### Optimasi RAM
- YOLOv8n (3M params) — paling ringan
- Inference 10Hz (timer 0.1s)
- Kamera bawah/samping infer tiap 5 frame
- C++ nodes (navigasi, motor) untuk loop responsif

### Hardware config (params.yaml)
| Komponen | Parameter | Default |
|----------|-----------|---------|
| Kamera utama | `device_utama` | `0` (/dev/video0) |
| Kamera bawah | `device_bawah` | `""` (none) |
| Kamera samping | `device_samping` | `""` (none) |
| LiDAR | `serial_port` | `""` (none) |
| Motor | `serial_port` | `""` (none) |
| Motor protocol | `protocol` | `raw` (raw/msp/mavlink) |

### Run
```bash
ros2 launch asv1 asv1.launch.py
```

### Build
```bash
colcon build --packages-select asv1
```

### Dependencies (new)
- `rplidar` (pip) — RPLidar C1 driver

## SESSION LOG — 17 June 2026 — Part 2 ✅ KINEMATIC CONTROL FIX

### Root Cause
- `VelocityControl` plugin doesn't move boats (no wheel friction on water)
- Custom C++ plugin setting `WorldPose` on model entity didn't update `dynamic_pose/info` — **PosePublisher reads `Pose` (local) component, NOT `WorldPose`**
- `CreateComponent` only works once; subsequent calls silently fail if component exists — must use `SetComponentData`
- `gz::transport::Node::Subscribe` callback works when node is a **member variable**, not a pointer

### The Fix
Custom plugin (`src/plugins/velocity_controller.cc` → `libvelocity_controller.so`):
- **Kinematic control**: sets `Pose` + `WorldPose` on model, `WorldPose` on link every PreUpdate
- **cmd_vel subscriber**: `gz::transport::Node::Subscribe` to `/model/{name}/cmd_vel`
- **No physics dependency**: works without ODE, gravity ignored (Pose override each frame)
- **Auto topic**: derives model name from `Name` component for dynamic topic path

### SDF Integration
`config/asv1_world.sdf`:
- Replaced `VelocityControl` plugin with `asv1::VelocityController`  
- `GZ_SIM_HEADLESS_MODE=1` env var in launch file needed for ogre2 sensors in headless

### Build
```bash
# Manual compile (not colcon):
cd src/asv1/src/plugins
g++ -shared -fPIC -o libvelocity_controller.so velocity_controller.cc \
  $(pkg-config --cflags --libs gz-sim8 gz-transport13 gz-msgs10) -std=c++17
```

### Run
```bash
GZ_SIM_HEADLESS_MODE=1 ros2 launch asv1 asv1_sim.launch.py
```

### Test (without ROS)
```bash
GZ_SIM_HEADLESS_MODE=1 gz sim -r /path/to/asv1_world.sdf
gz topic -t /model/kapal/cmd_vel -m gz.msgs.Twist -p "linear:{x:1} angular:{z:0.2}"
gz topic -e -t /world/asv1_world/dynamic_pose/info
```

### Known Issues (asv1)
- **Ogre2/EGL**: `GZ_SIM_HEADLESS_MODE=1` required for sensor rendering without GPU
- **Model**: `finallll_detek.pt` incompatible PyTorch 2.12; pakai `yolov8n.pt`
- **Kamera bawah/samping**: disabled by default (`device: ""`)
- **LiDAR**: dry mode kalo `/dev/ttyUSB0` gak konek
- **Motor**: dry mode kalo `/dev/ttyACM0`/`/dev/ttyUSB0` gak konek
- **Firebase**: butuh `firebase-key.json` di path yg dikonfigurasi

## SESSION LOG — 17 June 2026 — Part 4 ✅ MOTOR MAVLINK + FIREBASE INSTALL

### node_motor — C++ → Python + pymavlink
**Alasan:** SpeedyBee (ArduRover) pake MAVLink, bukan raw/MSP.

| Sebelum (C++) | Sesudah (Python) |
|---------------|-------------------|
| `src/node_motor.cpp` | `asv1/node_motor.py` |
| Raw/MSP/MAVLink manual | pymavlink (otomatis CRC, heartbeat, GPS parse) |
| Tanpa heartbeat | Heartbeat 1Hz (ArduPilot butuh ini) |
| GPS placeholder | Baca `GLOBAL_POSITION_INT` otomatis |

**MAVLink mapping:**
```
/cmd_vel linear.x → throttle (z)  [-1000..1000]
/cmd_vel angular.z → steering (x) [-1000..1000]
```
Kirim lewat `mavutil.mav.manual_control_send()`

### Firebase + Cloudinary
```bash
pip3 install firebase_admin cloudinary pymavlink --break-system-packages
```
Firebase butuh `firebase-key.json` — sesuaikan path di params.yaml.

### Config Motor (params.yaml)
```yaml
/node_motor:
  ros__parameters:
    serial_port: "/dev/ttyACM0"     # isi kalo FC udah konek
    protocol: "mavlink"
    max_linear: 0.5
    max_angular: 0.8
```

## SESSION LOG — 17 June 2026 — Part 5 ✅ LOGGING + VIEWER AUTO-LAUNCH

### Enhanced Logging
Semua node sekarang punya log informatif:

| Node | Log penting |
|------|------------|
| `node_kamera` | Konfigurasi kamera, resolusi aktual, frame count |
| `node_deteksi` | Model info, first frame, tracking offset, trigger event |
| `node_lidar` | Status koneksi, obstacle warning per sektor |
| `node_motor` | MAVLink connect, GPS fix, cmd_vel yg dikirim, battery |
| `node_navigasi` | Mode change (IDLE/OBSTACLE/VISUAL/GPS), params |
| `node_misi` | Camera received, trigger diterima, upload status |

Navigasi C++ sekarang nampilin MODE setiap kali priority berubah:
```
MODE: IDLE -> linear=0.00, angular=0.00
MODE: VISUAL -> linear=0.60, angular=0.15
MODE: OBSTACLE -> linear=0.30, angular=0.50
```

### Viewer Auto-Launch
Viewer (`viewer.py`) udah ditambah ke `asv1.launch.py` — otomatis muncul pas `ros2 launch`.

### langkah.md
File panduan lengkap di `~/robot_ws/langkah.md`:
- Cek port → edit config → run all → run satu-satu → debug → viewer

## SESSION LOG — 17 June 2026 — Part 6 ✅ PID CONTROL + AUTO-LOOP + LOGGING + SCRIPTS

### Changes Summary

**1. PID Control (node_navigasi.cpp)**
- Added I (integral) and D (derivative) terms for smoother tracking
- Added low-pass filter (lpf_alpha=0.3) on angular velocity — anti-wobble
- Anti-windup on integral term (clamped ±500)
- Reset PID state when not in VISUAL mode

**2. Auto-Loop Mission (node_navigasi.cpp)**
- State machine built into control loop: NORMAL → ARRIVED → RETURNING → NORMAL
- Triggers when `|offset| < 20px` for 5 consecutive seconds
- Stop for 2s → Reverse for 10s → Back to normal control
- Resets arrival counter when tracking lost or mode changes

**3. CSV Logging (node_navigasi.cpp)**
- Writes to `/tmp/navigasi_log.csv`
- Columns: `time_s, mode, offset_px, linear_x, angular_z, error_p, error_i, error_d`
- Auto-saves on shutdown

**4. Scripts (`~/robot_ws/scripts/`)**
- `run.sh` — `colcon build` + `ros2 launch asv1_sim.launch.py`
- `stop.sh` — kills all ROS2 + Gazebo processes
- `monitor.sh` — shows nodes, topics, tracking, cmd_vel, CSV line count

### Parameters (params_sim.yaml)
```yaml
visual_p_gain: 0.002    # proportional gain
visual_i_gain: 0.0001   # integral gain (anti-steady error)
visual_d_gain: 0.0005   # derivative gain (anti-wobble)
lpf_alpha: 0.3          # low-pass filter (0=no smoothing, 1=max smoothing)
```

### Run
```bash
# Option 1: one-shot
./scripts/run.sh

# Option 2: manual
cd ~/robot_ws && colcon build --packages-select asv1
source install/setup.bash
ros2 launch asv1 asv1_sim.launch.py

# Viewer (separate terminal)
source install/setup.bash && ros2 run asv1 viewer.py

# Stop
./scripts/stop.sh

# Monitor
./scripts/monitor.sh
```

## SESSION LOG — 17 June 2026 — Part 3 ✅ VIEWER & CONFIDENCE THRESHOLD

### Bug Fix — Decode ONNX Output (sigmoid ganda + BGR/RGB)
**Root cause:** `_decode_yolo()` apply sigmoid ke class scores yg UDAH sigmoid dari model output. Jadinya:
- Noise (raw ~0) → sigmoid(0) = 0.5 → keliatan confident palsu
- Deteksi real (raw ~0.9) → sigmoid(0.9) = 0.71 → confident turun drastis

**Kedua:** `frame` dari OpenCV BGR, tapi model training pake RGB → warna terbalik.

**Fix:**
1. Hapus `1.0/(1.0+np.exp(-cls_scores))` — pakai `cls_scores` langsung
2. Tambah `cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)` sebelum inference
3. Hasil: blank → 0 false positive, balls → conf 0.94+

### Viewer
- `scripts/viewer.py` → `ros2 run asv1 viewer.py`
- **1 window aja**: subscribe `/asv/vizu` (YOLO annotated frame dari node_deteksi)
- Cuma nampilin box RED (red_ball) + GREEN (green_ball) + midpoint tracking gate
- TIDAK nampilin raw kamera, target_surface/underwater, IGNORE ZONE, counter
- Tekan ESC untuk keluar

### Mengatur Confidence Threshold (tanpa rebuild)
- Edit `~/robot_ws/src/asv1/config/params.yaml` → `conf_threshold`
- Default: 0.6 (naik dari 0.3 karena terlalu sensitif, banyak false positive)
- Tinggal restart launch aja, params dibaca tiap startup

### Run dari awal
```bash
source ~/robot_ws/install/setup.bash
ros2 launch asv1 asv1.launch.py
```
Viewer YOLO otomatis muncul — gak perlu terminal terpisah.

### Model ONNX
- Hanya 2 class: green_ball (0), red_ball (1) — class 2/3 (target_surface/underwater) tdk ada di model
- Path: `/home/aji/robot_ws/finallll_detek.onnx` (di params.yaml)

## SESSION LOG — 17 June 2026 — Part 7 ✅ REAL DEPLOYMENT PREP

### params.yaml siap untuk kapal asli
- Added PID params: `visual_i_gain: 0.0001`, `visual_d_gain: 0.0005`, `lpf_alpha: 0.3`
- Speed turun: `cruise_speed: 0.5`, `approach_speed: 0.35` (aman di air)
- Protocol motor: `mavlink` (SpeedyBee/ArduRover)

### Checklist sebelum run di kapal asli
| Parameter | File | Lokasi | Isi |
|-----------|------|--------|-----|
| `serial_port` LiDAR | `params.yaml` | `/node_lidar` | `/dev/ttyUSB0` |
| `serial_port` Motor | `params.yaml` | `/node_motor` | `/dev/ttyACM0` |
| `device_utama` kamera | `params.yaml` | `/node_kamera` | `0` atau `/dev/video0` |
| `firebase-key.json` | `params.yaml` | `/node_misi` | pastikan file ada |

### Cek koneksi
```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/video*
```

### Run di kapal asli
```bash
source ~/robot_ws/install/setup.bash
ros2 launch asv1 asv1.launch.py
```

### Catatan real vs sim
| Aspek | Sim | Real |
|-------|-----|------|
| Kamera | Gazebo topic (`""`) | `/dev/video0` |
| LiDAR | dry mode | RPLidar C1 via USB |
| Motor | `protocol: raw` | `protocol: mavlink` via SpeedyBee |
| PID | tuned sim | perlu retune di air asli |
| Speed | 0.4 cruise | 0.5 cruise (bisa dinaikin gradual) |
| Auto-loop reverse | jalan di sim | mungkin kurang efektif di air (drift) |

## SESSION LOG — 19 June 2026 — ✅ SPEEDYBEE F405 WING MOTOR/SERVO FIX

### Root Cause
- **`manual_control` / `rc_channels_override` via MAVLink tidak berfungsi** — ArduRover di SpeedyBee F405 Wing tidak memproses MAVLink control input ke SERVO functions
- **`DO_SET_SERVO` langsung set PWM berfungsi** — mengirim MAV_CMD_DO_SET_SERVO langsung mengubah output SERVO
- **SERVO3, SERVO4 conflict timer ADC** — pin PA2/PA3 dipakai voltage/current sensor, tidak bisa PWM
- **S6/S7 conflict** — PC7/PC8 dipakai fungsi lain
- **Hanya SERVO1, SERVO2, SERVO5, SERVO8** yang output PWM

### Hardware Findings
| Pin | SERVO | DO_SET_SERVO(ch) | Status |
|:---:|:-----:|:----------------:|:------:|
| S1 | SERVO1 | ch=1 | ✅ Bisa PWM |
| S2 | SERVO2 | ch=2 | ✅ Bisa PWM |
| S3 | SERVO3 | ch=3 | ❌ Conflict ADC |
| S4 | SERVO4 | ch=4 | ❌ Conflict ADC |
| S5 | SERVO5 | ch=5 | ✅ Bisa PWM |
| S6 | SERVO6 | ch=6 | ❌ Conflict |
| S7 | SERVO7 | ch=7 | ❌ Conflict |
| S8 | SERVO8 | ch=8 | ✅ Bisa PWM (motor) |

**S1 dan S2 fisik rusak** — tidak output meskipun FC generate PWM.

### Final Wiring
| Fungsi | Pin | SERVO | DO_SET_SERVO |
|--------|:---:|:-----:|:------------:|
| Rudder kiri | **S1** (S2 rusak) | SERVO1 | ch=1 |
| Rudder kanan | **S5** | SERVO5 | ch=5 |
| Motor ESC | **S8** | SERVO8 | ch=8 |

Atau pake Y-cable di S5 buat dual rudder kalo S1 juga mati.

### Changes
- `node_motor.py`: ganti `manual_control_send` → `MAV_CMD_DO_SET_SERVO` langsung
- Mapping: steer → ch=1 + ch=5, throttle → ch=8
- DTR reset sebelum MAVLink connect (fix serial race condition)
- SERVO functions di FC semuanya disable (`SERVO*_FUNCTION=0`) — biar DO_SET_SERVO override penuh

### Output Mapping (node_motor.py)
```python
_steer_ch1 = 1   # S1 (SERVO1)
_steer_ch2 = 5   # S5 (SERVO5)
_throttle_ch = 8 # S8 (SERVO8)
```

### PWM Range
- 1500 = neutral/stop
- 1100-1900 = full range (dibatasi di code)
- steer: 1500 ± (angular/max_angular)*400
- throttle: 1500 ± (linear/max_linear)*400
