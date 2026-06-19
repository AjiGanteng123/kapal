# Saran Perbaikan (Future Work)

## B. Tingkatkan Sistem

### 1. Tuning PID / Control
- `cruise_speed`: biar kapal lebih smooth ngejar midpoint
- `visual_p_gain`: responsiveness tracking
- Tambah integral/derivative term (PID instead of P)
- Smoothing angular velocity (low-pass filter)

### 2. Obstacle Avoidance
- Integrasi LiDAR obstacle ke node_navigasi
- Kapal berhenti/hindar kalo ada halangan di depan
- Sensor fusion: LiDAR + visual untuk deteksi halangan

### 3. Auto-Restart / Loop Mission
- Setelah sampe di bola, reset posisi atau jalan ke waypoint berikutnya
- State machine: SEARCH → TRACK → ARRIVE → RESET
- Timer-based mission cycle

### 4. GPS Waypoint Fallback
- Kalo bola gak keliatan (tracking status=0), kapal balik ke waypoint terakhir
- Bearing-to-waypoint navigation sebagai mode default
- Waypoint list di params.yaml

### 5. Logging ke File
- Record tracking offset, pose, cmd_vel tiap step ke CSV
- Buat analisis performa setelah run
- Plot trajectory vs time

### 6. Parameter Tuning
- `conf_threshold`: sensitivity deteksi
- HSV thresholds untuk color fallback
- Frame skip (utama tiap frame, bawah/samping tiap 5 frame)
- Resolusi kamera (640x480 vs 320x240 untuk performa)

## C. Dokumentasi / Cleanup

### 1. Rapihin Kode
- Hapus commented-out code
- Standarisasi logging format
- Type hints di Python nodes
- Error handling yang lebih baik

### 2. Script One-Shot Run
- `./run_sim.sh` — build + launch dalam 1 perintah
- `./stop.sh` — kill semua proses ROS2 + Gazebo
- `./monitor.sh` — tampilkan semua topic + log

### 3. README / Docs
- Architecture diagram
- Topic list
- Cara konfigurasi hardware (kamera, LiDAR, motor)
- Troubleshooting guide

### 4. Test & Validation
- Unit test untuk detection logic
- Simulation test dengan known positions
- Edge cases: bola out-of-frame, lighting changes, multiple objects
