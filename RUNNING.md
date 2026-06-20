# RUNNING.md — ASV1 Robot Operation

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_ws && colcon build --packages-select asv1
source install/setup.bash
```

> Ulangi `colcon build` setiap ada perubahan kode.

## Cek Port Hardware

```bash
ls -la /dev/ttyACM* /dev/ttyUSB* /dev/video*
# /dev/ttyUSB0 = RPLidar
# /dev/ttyACM0 = SpeedyBee
# /dev/video0  = Kamera
```

Edit port di `~/robot_ws/src/asv1/config/params.yaml` kalo beda.

## Run Semua Node

```bash
source ~/robot_ws/install/setup.bash
ros2 launch asv1 asv1.launch.py
```

Atau dengan script:
```bash
./scripts/run.sh
```

### Run Satu-Satu (Debug)

```bash
source ~/robot_ws/install/setup.bash

ros2 run asv1 node_kamera.py     # Kamera
ros2 run asv1 node_deteksi.py    # YOLO detection
ros2 run asv1 node_lidar.py      # LiDAR obstacle
ros2 run asv1 node_navigasi      # Navigator C++
ros2 run asv1 node_motor.py      # Motor MAVLink
ros2 run asv1 node_misi.py       # Firebase mission
ros2 run asv1 viewer.py          # Viewer YOLO
```

## Stop

```bash
pkill -f "ros2|asv1|python3"  # atau
./scripts/stop.sh
```

## Cek Data

```bash
ros2 topic echo /asv/obstacle   # 4 sektor jarak obstacle
ros2 topic echo /asv/tracking   # Midpoint red+green ball
ros2 topic echo /asv/cmd_vel    # Perintah ke motor
ros2 topic echo /asv/telemetri   # [lat, lon, hdg, heading, speed, battery]
ros2 topic list                  # Semua topic
```

## Priority Control

Navigasi otomatis berdasarkan prioritas:
1. **LiDAR** — stop/turn kalo obstacle < 0.4m
2. **YOLO** — steer ke midpoint red+green ball
3. **GPS** — bearing ke waypoint (fallback)

## Prasyarat Hardware

| Komponen | Port | Parameter |
|----------|------|-----------|
| LiDAR | `/dev/ttyUSB0` | `/node_lidar/serial_port` |
| SpeedyBee | `/dev/ttyACM0` | `/node_motor/serial_port` |
| Kamera | `/dev/video0` | `/node_kamera/device_utama` |

### SpeedyBee FC Settings
- Mode **GUIDED** (4) — wajib biar `cmd_vel` diterima
- `SERVO1_FUNCTION=0`, `SERVO5_FUNCTION=0`, `SERVO8_FUNCTION=0`
- Wajib **ARMED** (auto-arm via node_motor)
- Cek mode: `ros2 topic echo /asv/fc_mode`

## Simulasi (Gazebo)

```bash
source ~/robot_ws/install/setup.bash
ros2 launch asv1 asv1_sim.launch.py
```

## Parameter Config

File: `~/robot_ws/src/asv1/config/params.yaml`

| Parameter | Default | Fungsi |
|-----------|---------|--------|
| `conf_threshold` | 0.6 | Sensitivity YOLO (0.3=sensitif, 0.8=sangat yakin) |
| `serial_port` (motor) | `/dev/ttyACM0` | Port SpeedyBee |
| `protocol` | `mavlink` | mavlink / msp / raw |
| `device_utama` | 0 | Kamera utama |

## Topics

| Topic | Type | Publisher |
|-------|------|-----------|
| `/asv/kamera/utama` | Image | node_kamera |
| `/asv/tracking` | Float32MultiArray | node_deteksi |
| `/asv/obstacle` | Float32MultiArray | node_lidar |
| `/asv/cmd_vel` | Twist | node_navigasi |
| `/asv/telemetri` | Float32MultiArray | node_motor |
| `/asv/fc_mode` | Int32 | node_motor |
| `/asv/vizu` | Image | node_deteksi |
| `/asv/trigger` | Int32 | node_deteksi |
| `/scan` | LaserScan | RPLidar |

## Catatan

- Hardware gak konek → dry mode (sistem tetap jalan, skip node error)
- Ganti `conf_threshold` → restart launch, gak perlu build
- Ganti kode C++ → `colcon build` dulu
- Model ONNX: `/home/aji/robot_ws/finallll_detek.onnx`
- Kamera bawah/samping: set `device: ""` untuk disable
