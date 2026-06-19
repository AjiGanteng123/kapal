# Langkah-Langkah Operasi Kapal ASV

---

## 1. Build (kalau ada perubahan kode)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_ws
colcon build --packages-select asv1
source install/setup.bash
```

---

## 2. Cek Port Hardware

Colok RPLidar C1 & SpeedyBee (ArduRover) ke USB laptop.

```bash
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Biasanya keluar:
- `/dev/ttyUSB0` → RPLidar
- `/dev/ttyACM0` → SpeedyBee

Kalo kosong, cek: `dmesg | tail -20`

---

## 3. Edit Konfigurasi

```bash
nano ~/robot_ws/src/asv1/config/params.yaml
```

### Bagian yg diisi sesuai hardware:

```yaml
/node_lidar:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"       # port RPLidar

/node_motor:
  ros__parameters:
    serial_port: "/dev/ttyACM0"       # port SpeedyBee
    protocol: "mavlink"
```

### Bagian threshold (atur sesuai kondisi):

```yaml
/node_deteksi:
  ros__parameters:
    conf_threshold: 0.6               # 0.3=sensitif, 0.6=standar, 0.8=sangat yakin
```

Simpan: `Ctrl+X` → `Y` → `Enter`

---

## 4. Run Semua Sekaligus

```bash
source ~/robot_ws/install/setup.bash
ros2 launch asv1 asv1.launch.py
```

Ini jalanin: kamera, YOLO, LiDAR, navigasi, motor, misi.

---

## 5. Run Satu-Satu (buat debug)

Buka 1 terminal per node, masing-masing source dulu:

```bash
source ~/robot_ws/install/setup.bash
```

| Terminal | Perintah | Fungsi |
|----------|----------|--------|
| 1 | `ros2 run asv1 node_kamera.py` | Baca kamera utama |
| 2 | `ros2 run asv1 node_deteksi.py` | YOLO detection |
| 3 | `ros2 run asv1 node_lidar.py` | LiDAR obstacle |
| 4 | `ros2 run asv1 node_navigasi` | Navigator |
| 5 | `ros2 run asv1 node_motor.py` | Motor MAVLink |
| 6 | `ros2 run asv1 node_misi.py` | Firebase mission |
| 7 | `ros2 run asv1 viewer.py` | Lihat YOLO |

---

## 6. Cek Data

```bash
source ~/robot_ws/install/setup.bash

# Lihat jarak obstacle 4 sektor (depan, kanan, belakang, kiri)
ros2 topic echo /asv/obstacle

# Lihat tracking YOLO (midpoint red+green ball)
ros2 topic echo /asv/tracking

# Lihat perintah gerak ke motor
ros2 topic echo /asv/cmd_vel

# Lihat telemetri GPS
ros2 topic echo /asv/telemetri

# Semua topic aktif
ros2 topic list
```

---

## 7. Viewer YOLO

Viewer otomatis jalan pas `ros2 launch asv1 asv1.launch.py`.
Window nampilin deteksi RED/GREEN + tracking gate. **ESC** untuk keluar.

---

## 8. Priority Control

Navigasi otomatis pake urutan prioritas:

1. **LiDAR** — stop/turn kalo obstacle < 0.4m di depan
2. **YOLO** — steer ke midpoint antara red + green ball
3. **GPS** — bearing ke waypoint (fallback)

---

## 9. Catatan

- Hardware gak konek → dry mode (sistem tetap jalan, motor/lidar skip)
- Ganti `conf_threshold` → **restart launch**, gak perlu build ulang
- Ganti kode → **colcon build dulu**, baru run


mono MissionPlanner.exe

node navigasi cpp: tambah scan mode, cari bola kanan kiri.
node motor.py rc mode awareness. manual/rtl/guided
bide_trainer.py: training data recorder (lidar+cmd_vel
node_lidar.py enable raw csv recording
config: params.yaml  + launch file update
build and verify.

untuk chanel ini di speeyby motor ch 5, servo ch 1 dan 8
