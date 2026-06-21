# Capture Logic — Kamera 2 (Samping)

## Flow

```
Kamera 2 (/dev/video1) → /asv/kamera/samping → node_deteksi
                                                    ↓
                                         YOLO detect target_surface (class 2)
                                                    ↓
                                            _validate_capture():
                                              1. confidence ≥ 0.8
                                              2. objek di tengah (25%-75% frame)
                                              3. area ≥ 2000 px
                                              4. tidak blur (Laplacian ≥ 100)
                                                    ↓
                                           5 frame berturut-turut lolos?
                                                    ↓
                                           trigger=1 → /asv/trigger
                                                    ↓
                                           node_misi:
                                              - ambil frame dari kamera samping
                                              - upload ke Cloudinary
                                              - push URL ke Firebase
                                                    ↓
                                                   Web
```

## Dependency

```
node_kamera → node_deteksi → node_misi → Firebase + Cloudinary
```

## Validasi Capture (`node_deteksi.py:254-281`)

| Check | Param | Default | Ketentuan |
|-------|-------|---------|-----------|
| Confidence | `capture_conf_threshold` | 0.8 | YOLO confidence ≥ threshold |
| Posisi | `capture_position_margin` | 0.25 | cx/cy dalam 25%-75% frame |
| Ukuran | `capture_min_area` | 2000 | Luas bounding box ≥ threshold |
| Blur | `capture_blur_threshold` | 100.0 | Laplacian variance ≥ threshold |

## Trigger (`node_deteksi.py:427-433`)

Hanya trigger kalo validasi lolos 5 frame berturut-turut (`required_frames: 5`).

## Config (`params.yaml`)

```yaml
/node_kamera:
  ros__parameters:
    device_samping: 1           # kamera 2

/node_deteksi:
  ros__parameters:
    capture_conf_threshold: 0.8
    capture_min_area: 2000
    capture_blur_threshold: 100.0
    capture_position_margin: 0.25
```
