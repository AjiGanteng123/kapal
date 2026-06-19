# Setup & Run — ASV1 Robot

## 1. Prasyarat

```bash
# ROS2 Jazzy sudah terinstall
source /opt/ros/jazzy/setup.bash
```

## 2. Build

```bash
cd ~/robot_ws
colcon build --packages-select asv1
source install/setup.bash
```

> Ulangin `colcon build` setiap ada perubahan kode Python/C++.

## 3. Konfigurasi Hardware

Edit `config/params.yaml`:

```bash
nano ~/robot_ws/src/asv1/config/params.yaml
```

| Parameter | Default | Isi dengan |
|-----------|---------|------------|
| `/node_kamera/device_utama` | `0` | `/dev/video0` |
| `/node_lidar/serial_port` | `""` | `/dev/ttyUSB0` |
| `/node_motor/serial_port` | `""` | `/dev/ttyACM0` |
| `/node_motor/protocol` | `raw` | `raw` / `msp` / `mavlink` |

Cek port dulu:
```bash
ls /dev/tty*
```

## 4. Run

### Simulasi (Gazebo)

```bash
source ~/robot_ws/install/setup.bash
ros2 launch asv1 asv1_sim.launch.py
```

> GUI Gazebo akan muncul. Boat otomatis jalan (visual tracking).
> Cek posisi: `timeout 2 gz topic -e -t /world/asv1_world/pose/info -n 1 | grep "kapal" -A4`

### Kapal Asli

```bash
source ~/robot_ws/install/setup.bash
ros2 launch asv1 asv1.launch.py
```

> Pastikan hardware (kamera, LiDAR, serial) sudah terhubung.

## 5. Topik ROS2 Penting

| Topic | Type | Fungsi |
|-------|------|--------|
| `/asv/cmd_vel` | Twist | Perintah gerak (dari node_navigasi) |
| `/asv/kamera/utama` | Image | Feed kamera utama |
| `/asv/obstacle` | Float32MultiArray | 4 sektor jarak obstacle |
| `/asv/tracking` | Float32MultiArray | Posisi target bola (midpoint x,y) |
| `/asv/telemetri` | Float32MultiArray | [lat, lon, hdg, heading, speed, battery] |
| `/scan` | LaserScan | Raw LiDAR |

## 6. Debug

```bash
# Lihat semua topic
ros2 topic list

# Echo cmd_vel
ros2 topic echo /asv/cmd_vel

# Lihat posisi (sim)
gz topic -e -t /world/asv1_world/dynamic_pose/info

# Kirim perintah manual
ros2 topic pub /asv/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}" -1
```

## 7. Catatan

- **GPU**: Butuh NVIDIA driver buat rendering Gazebo. Kalo software render, visual freeze.
- **RAM**: 8GB cukup. YOLOv8n (3M params) paling ringan.
- **Headless**: Kalo GPU error, bisa set `GZ_SIM_HEADLESS_MODE=1` (sensor tetep jalan, visual freeze).
