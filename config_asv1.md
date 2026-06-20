# Config ASV1 — PID & Hardware

## PID Navigator (`params.yaml` → `/node_navigasi`)

| Parameter | Nilai | Fungsi |
|-----------|-------|--------|
| `visual_p_gain` | 0.0015 | Proportional — keras ngejar offset |
| `visual_i_gain` | 0.0001 | Integral — ilangin error sisa |
| `visual_d_gain` | 0.001 | Derivative — nahan overshoot |
| `lpf_alpha` | 0.4 | Low-pass filter (0=kasar, 1=halus) |
| `max_yaw_rate` | 0.45 | Belok maks (rad/s) |
| `cruise_speed` | 0.5 | Kecepatan jelajah |
| `approach_speed` | 0.35 | Kecepatan pas tracking |

## PID Tuning Guide

| Skenario | P | I | D | LPF | max_yaw | cruise |
|----------|---|---|---|-----|---------|--------|
| Halus | 0.0008 | 0.00003 | 0.002 | 0.6 | 0.25 | 0.35 |
| Sedang | 0.0012 | 0.00005 | 0.0015 | 0.5 | 0.3 | 0.4 |
| **Sekarang** | **0.0015** | **0.0001** | **0.001** | **0.4** | **0.45** | **0.5** |
| Agresif | 0.002 | 0.00015 | 0.0005 | 0.3 | 0.5 | 0.5 |

Edit di `~/robot_ws/src/asv1/config/params.yaml`.

## Hardware Mapping (DO_SET_SERVO)

| Fungsi | Pin | SERVO | ch |
|--------|:---:|:-----:|:--:|
| Motor ESC | S8 | SERVO8 | 8 |
| Rudder kanan | S5 | SERVO5 | 5 |
| Rudder kiri | S2 | SERVO2 | 2 |
| (cadangan) | S1 | SERVO1 | 1 |

## Obstacle / Fusion

| Parameter | Nilai | Fungsi |
|-----------|-------|--------|
| `obstacle_stop_dist` | 0.4 | Jarak stop (meter) |
| `obstacle_avoid_speed` | 0.3 | Kecepatan hindar |
| `fusion_dist_min/max` | 0.3 / 3.0 | Range fusion LiDAR+visual |

## File

| File | Lokasi |
|------|--------|
| Config | `~/robot_ws/src/asv1/config/params.yaml` |
| Motor node | `~/robot_ws/src/asv1/asv1/node_motor.py` |
| Navigator | `~/robot_ws/src/asv1/src/node_navigasi.cpp` |
| Launch | `~/robot_ws/src/asv1/launch/asv1.launch.py` |
