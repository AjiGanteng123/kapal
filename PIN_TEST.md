# PIN TESTING — SpeedyBee F405 Wing

Test pin servo satu per satu pakai DO_SET_SERVO via MAVLink.

## 1. Setup

Colok servo ke S1 (signal), BEC 5V (power), GND.

```bash
ls /dev/ttyACM*
# Pastikan ada /dev/ttyACM0
```

## 2. Disable Semua SERVO_FUNCTION

```bash
python3 << 'EOF'
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0', 115200)
m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
for ch in range(1, 9):
    m.mav.param_set_send(m.target_system, 1, f'SERVO{ch}_FUNCTION'.encode(), 0.0, 9)
    print(f'SERVO{ch}_FUNCTION=0')
EOF
```

## 3. Test S1 sampai S8 (Loop)

```bash
python3 << 'EOF'
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('/dev/ttyACM0', 115200)
m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
for ch in range(1, 9):
    print(f'\n=== S{ch} ===')
    m.mav.command_long_send(m.target_system, 1, 183, 0, ch, 1500,0,0,0,0,0); time.sleep(1)
    m.mav.command_long_send(m.target_system, 1, 183, 0, ch, 1200,0,0,0,0,0); print('1200us (MIN)'); time.sleep(2)
    m.mav.command_long_send(m.target_system, 1, 183, 0, ch, 1800,0,0,0,0,0); print('1800us (MAX)'); time.sleep(2)
    m.mav.command_long_send(m.target_system, 1, 183, 0, ch, 1500,0,0,0,0,0); print('1500us (NEUTRAL)')
    input('Pindah kabel ke pin berikutnya, lalu Enter')
EOF
```

## 4. Test Satu Pin

```bash
CH=2 python3 << 'EOF'
from pymavlink import mavutil; import time; import os
ch = int(os.environ['CH'])
m = mavutil.mavlink_connection('/dev/ttyACM0', 115200)
m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
print(f'Test S{ch}')
m.mav.command_long_send(m.target_system, 1, 183, 0, ch, 1500,0,0,0,0,0); time.sleep(1)
m.mav.command_long_send(m.target_system, 1, 183, 0, ch, 1200,0,0,0,0,0); time.sleep(2)
m.mav.command_long_send(m.target_system, 1, 183, 0, ch, 1800,0,0,0,0,0); time.sleep(2)
m.mav.command_long_send(m.target_system, 1, 183, 0, ch, 1500,0,0,0,0,0)
print(f'S{ch} done')
EOF
```

Ganti `CH=2` jadi pin yang mau dites.

## 5. Hasil

| PIN | Status | Catatan |
|:---:|:------:|---------|
| S1 | | |
| S2 | | |
| S3 | | Conflict ADC (PA2/PA3) |
| S4 | | Conflict ADC |
| S5 | | |
| S6 | | Conflict |
| S7 | | Conflict |
| S8 | | |

✅ = gerak, ❌ = diam, ⚠️ = jitter/conflict

## Catatan

- S3/S4 conflict ADC (voltage/current sensor) — jangan dipake
- S6/S7 conflict fungsi lain — jangan dipake
- Ganti 1200/1800 jadi 1100/1900 untuk ESC full range
- Kalo port error, coba `/dev/ttyACM1`
