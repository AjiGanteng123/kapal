# Instruksi Operasi Robot

## 1. Setup Awal

```bash
# Build workspace
cd ~/robot_ws
colcon build --packages-select robot_stack

# Source workspace (tiap buka terminal baru)
source install/setup.bash
```

## 2. Install Dependencies (sekali saja)

```bash
# kalau pake YOLO
pip3 install ultralytics torch

# kalau pake MAVLink (SpeedyBee F405 ArduPilot)
pip3 install pymavlink

# Gazebo (kalau belum ada)
sudo apt install ros-jazzy-gazebo-ros-pkgs
```

## 3. Jalankan Semua Node

```bash
source ~/robot_ws/install/setup.bash
ros2 launch robot_stack robot_stack.launch.py
```

Atau jalankan satu per satu (debug):

```bash
ros2 run robot_stack lidar_node
ros2 run robot_stack vision_node
ros2 run robot_stack autonomous_node
ros2 run robot_stack motor_node
```

## 4. Konfigurasi

Edit parameter di `src/robot_stack/config/params.yaml`:

| Parameter | Fungsi |
|-----------|--------|
| `scan_topic` | Topic LiDAR |
| `camera_topic` | Topic kamera |
| `model_path` | Path ke model YOLO (isi kalo punya .pt) |
| `serial_port` | Port SpeedyBee (`/dev/ttyACM0` atau `/dev/ttyUSB0`) |
| `protocol` | `mavlink`, `msp`, atau `raw` |
| `forward_speed` | Kecepatan maju (m/s) |
| `turn_speed` | Kecepatan belok (rad/s) |

Kosongin `model_path` kalo mau pake deteksi warna default (hijau/merah).

## 5. Cara Kerja Robot

1. **SEARCH** — robot muter nyari bola hijau
2. **APPROACH_GREEN** — mendekati bola hijau
3. **PASS_THROUGH** — kalo liat bola hijau + merah, robot jalan di tengah2nya
4. **APPROACH_RED** — mendekati bola merah
5. Otomatis balik ke SEARCH kalo udah lewat

## 6. SpeedyBee F405

- Default MAVLink di `/dev/ttyACM0` 115200 baud
- Kalo pake Betaflight (MSP), ganti `protocol: "msp"` di params.yaml
- Coba `ls /dev/tty*` buat lihat port serial
- Kalo perlu: `sudo chmod 666 /dev/ttyACM0` (atau tambah user ke grup dialout)

## 7. Lihat Data

```bash
# Lihat obstacle (4 sector: depan, kanan, belakang, kiri)
ros2 topic echo /robot/obstacle

# Lihat deteksi [tipe, cx, cy, bw, bh, conf, ...]
ros2 topic echo /robot/detections

# Lihat perintah ke motor
ros2 topic echo /robot/cmd_vel

# Lihat semua topic
ros2 topic list
```

## 8. Gazebo Ship

Model kapal ada di `models/kapal/`. Cara spawn di Gazebo:

```bash
# 1) set model path
export GAZEBO_MODEL_PATH=$HOME/robot_ws/models:$GAZEBO_MODEL_PATH

# 2) jalankan gazebo
gazebo

# 3) di GUI Gazebo: Insert -> pilih "Kapal"
```

Atau via terminal (Gazebo Classic):
```bash
source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH=$HOME/robot_ws/models:$GAZEBO_MODEL_PATH
gz model --spawn-file=$HOME/robot_ws/models/kapal/model.sdf --model-name=kapal -x 2 -y 0 -z 0
```

Model kapal terdiri dari beberapa bagian:
- **Hull** (biru) — box 2m x 0.6m x 0.3m
- **Cabin** (putih) — box kecil di atas
- **Funnel** (merah) — cerobong asap
- **Bow** (kuning) — haluan kapal

## 9. Cek SpeedyBee Connection

```bash
# cek port
ls -la /dev/ttyACM* /dev/ttyUSB*

# test serial
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0',115200); print('OK')"
```
