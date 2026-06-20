# Tutorial Lengkap: LiDAR RPLIDAR C1 + SLAM Mapping

## Prasyarat
- ROS2 Jazzy terinstall
- Workspace `~/robot_ws` sudah di-build
- RPLIDAR C1 terhubung via USB

---

## 1. Cek Port LiDAR

```bash
ls /dev/ttyUSB*
```

Hasil: biasanya `/dev/ttyUSB0` atau `/dev/ttyUSB1`.

---

## 2. Jalankan Semua via tmux

Cukup **satu terminal**, semua proses jalan di background:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

# ----- 2a. LiDAR -----
tmux new-session -d -s lidar
tmux send-keys -t lidar 'source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash && ros2 launch rplidar_ros rplidar_c1_launch.py serial_baudrate:=460800 serial_port:=/dev/ttyUSB0' Enter

sleep 1

# ----- 2b. TF odom → base_link -----
tmux new-session -d -s tf_odom
tmux send-keys -t tf_odom 'source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link' Enter

sleep 1

# ----- 2c. TF base_link → laser -----
tmux new-session -d -s tf_laser
tmux send-keys -t tf_laser 'source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser' Enter

sleep 1

# ----- 2d. SLAM Mapping -----
tmux new-session -d -s slam
tmux send-keys -t slam 'source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:=~/robot_ws/src/asv1/config/slam_params.yaml' Enter

sleep 4

# ----- 2e. RViz Visualisasi -----
tmux new-session -d -s rviz
tmux send-keys -t rviz 'source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash && rviz2 -d ~/robot_ws/src/asv1/config/slam_view.rviz' Enter

sleep 3
echo "=== SEMUA JALAN ==="
tmux ls
```

**Kalo port bukan ttyUSB0**, ganti di bagian (2a):
```
serial_port:=/dev/ttyUSB1
```

---

## 3. Navigasi tmux

| Perintah | Fungsi |
|----------|--------|
| `tmux attach -t lidar` | Lihat log LiDAR |
| `tmux attach -t slam` | Lihat log SLAM |
| `tmux attach -t rviz` | Lihat log RViz |
| `Ctrl+B` lalu `D` | Detach dari session (kembali ke terminal) |

---

## 4. Visualisasi di RViz

Setelah RViz muncul, akan terlihat:
- **Peta abu-abu** = area yang belum dipetakan
- **Titik hijau** = data LiDAR real-time
- **Grid** = referensi koordinat

> Jika layar hitam: di RViz, set **Fixed Frame** dropdown (pojok kiri atas) ke `map`.

**Navigasi RViz:**
- Klik kiri + drag = putar view
- Scroll = zoom in/out
- Klik kanan + drag = geser view

---

## 5. Kumpulkan Data Mapping

Gerakkan LiDAR (atau robot) secara perlahan.

**Tips:**
- Gerakan pelan dan stabil
- Overlap antar scan membantu akurasi
- Loop closure (balik ke posisi awal) bikin peta lebih rapi
- Pantau di RViz: titik hijau = scan, hitam = obstacle terdeteksi

---

## 6. Simpan Peta

Kalo udah puas, di terminal mana saja:

```bash
source ~/robot_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/peta_ruangan
```

Hasil:

| File | Isi |
|------|-----|
| `~/robot_ws/peta_ruangan.yaml` | Metadata (resolusi, origin, path) |
| `~/robot_ws/peta_ruangan.pgm` | Gambar peta (format PGM, harus dikonversi) |

Kalo mau lihat gambarnya langsung:

```bash
# Konversi PGM ke PNG biar bisa dibuka
python3 -c "
from PIL import Image
img = Image.open('$HOME/robot_ws/peta_ruangan.pgm')
img.save('$HOME/robot_ws/peta_ruangan.png')
"
xdg-open ~/robot_ws/peta_ruangan.png
```

Atau kalo gak punya PIL:

```bash
# Install dulu
pip install Pillow --break-system-packages
```

---

## 7. Lihat Hasil Peta yang Sudah Disimpan

### 7a. Cek Statistik Peta

```bash
source ~/robot_ws/install/setup.bash

# Info metadata
cat ~/robot_ws/peta_ruangan.yaml

# Statistik pixel
python3 -c "
with open('/home/aji/robot_ws/peta_ruangan.pgm', 'rb') as f:
    for _ in range(3):
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
    w, h = map(int, line.split())
    data = f.read()
    
gray = data.count(bytes([205]))
white = data.count(bytes([254]))
black = data.count(bytes([0]))
total = len(data)
print(f'Ukuran: {w}x{h} = {w*0.05:.1f}m x {h*0.05:.1f}m')
print(f'Unknown (abu-abu): {gray} ({100*gray/total:.1f}%)')
print(f'Free (putih): {white} ({100*white/total:.1f}%)')
print(f'Obstacle (hitam): {black} ({100*black/total:.1f}%)')
"
```

**Target peta bagus:** obstacle > 10%, unknown < 50%.

### 7b. Load Peta Lama ke RViz

Matikan SLAM dulu (peta baru gantian):

```bash
tmux kill-session -t slam
```

Lalu publish peta yang udah disimpan:

```bash
source ~/robot_ws/install/setup.bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/robot_ws/peta_ruangan.yaml
```

Di RViz tinggal subscribe ke `/map` (otomatis kalo config `slam_view.rviz`).

---

## 8. Mapping Ulang Lebih Ekstensif

Kalo peta pertama kurang detail, ulangi mapping dengan gerakan lebih luas.

### Langkah:
1. Stop SLAM: `tmux kill-session -t slam`
2. Hapus peta lama (biar mulai dari nol):
   ```bash
   rm ~/robot_ws/peta_ruangan.yaml ~/robot_ws/peta_ruangan.pgm
   ```
3. Jalankan ulang SLAM (langkah 2d)
4. **Gerakkan LiDAR secara sistematis:**

```
    ╔══════════════════╗
    ║   ─────→         ║
    ║   ←─────         ║
    ║   ─────→         ║
    ║   ←─────         ║
    ╚══════════════════╝
```

   Gerakan zig-zag, pastikan tiap area tersapu scan LiDAR.

5. Pantau di RViz — peta hitam-putih akan terbentuk
6. Simpan: `ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/peta_ruangan_v2`

### Tips Mapping Luas:
| Prinsip | Penjelasan |
|---------|------------|
| **Gerakan zig-zag** | Sapu seluruh area seperti vacuum cleaner |
| **Overlap 30%** | Setiap baris scan harus tumpang tindih |
| **Loop closure** | Kembali ke posisi awal biar peta rapi |
| **Hindari putaran cepat** | LiDAR butuh waktu 0.1s per scan |
| **Jarak ke dinding** | Maks 16m, usahakan < 8m untuk akurasi |
| **Objek transparan** | Kaca / air tidak terbaca LiDAR (inf) |

---

## 9. Setting Deteksi Obstacle (Biar Hitamnya Keliatan)

Kalo obstacle di peta kurang kebaca, tune parameter ini di `~/robot_ws/src/asv1/config/slam_params.yaml`:

### Parameter Utama

| Parameter | Default | Setting Baru | Efek |
|-----------|---------|--------------|------|
| `occupancy_threshold` | 0.1 | **0.05** | Lebih sensitif — obstacle tipis pun kebaca |
| `min_laser_range` | 0.15 | **0.1** | Tangkap objek lebih dekat |
| `minimum_travel_distance` | 0.1 | **0.05** | Update peta walau gerak dikit |
| `minimum_travel_heading` | 0.1 | **0.05** | Update peta walau muter dikit |
| `link_match_minimum_response_fine` | 0.1 | **0.05** | Scan matching lebih agresif |
| `min_pass_through` | 2 | **1** | 1 scan udah cukup buat mark obstacle |
| `scan_buffer_size` | 20 | **30** | Lebih banyak scan buat referensi |

### Scan Mode LiDAR

Tambahkan `scan_mode:=DenseBoost` di langkah 2a:

```bash
# Sebelum:
ros2 launch rplidar_ros rplidar_c1_launch.py serial_baudrate:=460800 serial_port:=/dev/ttyUSB0

# Sesudah (lebih padat, 40m):
ros2 launch rplidar_ros rplidar_c1_launch.py serial_baudrate:=460800 serial_port:=/dev/ttyUSB0 scan_mode:=DenseBoost
```

### Cara apply:

```bash
# 1. Stop SLAM
tmux kill-session -t slam

# 2. Edit params (occupancy_threshold dll) sesuai tabel di atas
nano ~/robot_ws/src/asv1/config/slam_params.yaml

# 3. Start ulang SLAM (langkah 2d)
# 4. Mapping ulang
```

---

## 10. Hentikan Semua

```bash
tmux kill-session -t lidar
tmux kill-session -t tf_odom
tmux kill-session -t tf_laser
tmux kill-session -t slam
tmux kill-session -t rviz
```

Atau sekali tebas:
```bash
tmux kill-server
```

---

## Troubleshooting

| Masalah | Solusi |
|---------|--------|
| `Error, code: 80008004` | Port USB dipakai. `pkill -f rplidar` lalu ulangi |
| Port berubah (`ttyUSB0` → `ttyUSB1`) | Pakai port baru di langkah 2a |
| RViz hitam / titik hilang | Fixed Frame → `map` atau klik **Reset** di RViz |
| Map abu-abu semua | LiDAR belum digerakkan |
| `ros2: command not found` | Belum `source /opt/ros/jazzy/setup.bash` |

---

## File Penting

| File | Lokasi |
|------|--------|
| Parameter SLAM | `~/robot_ws/src/asv1/config/slam_params.yaml` |
| Konfigurasi RViz | `~/robot_ws/src/asv1/config/slam_view.rviz` |
| Panduan ini | `~/robot_ws/lidar_slam_guide.md` |
| Hasil peta v1 | `~/robot_ws/peta_ruangan.yaml` + `peta_ruangan.pgm` |
| Hasil peta v2 | `~/robot_ws/peta_ruangan_v2.yaml` + `peta_ruangan_v2.pgm` |
