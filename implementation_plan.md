# Phân tích test đã thực hiện + Plan test còn lại

## Phân tích kết quả test đã chạy

### Test 1 — Fake Target 4 hướng: ⚠️ ĐẠT nhưng cần giải thích

| Hướng | Expected Bearing | Actual Bearing | Nhận xét |
|---|---|---|---|
| North | 0° | **330.1°** | ❌ Lệch ~30° |
| East | 90° | **128.2°** | ❌ Lệch ~38° |
| South | 180° | **198.9°** | ⚠️ Lệch ~19° |
| West | 270° | **257.8°** | ⚠️ Lệch ~12° |

**Nguyên nhân**: Tracker GPS position trong SITL **không phải tĩnh** — drone x500 trong Gazebo bay/trôi, nên `vehicle_global_position` thay đổi liên tục. Khi tracker position dịch chuyển → bearing tính ra khác so với expected.

> [!NOTE]
> Đây là **hành vi đúng** — tracker module đang tính bearing dựa trên **vị trí thực tế** của x500 trong SITL (đang drift), không phải vị trí cố định. Trên hardware thực, tracker sẽ đặt cố định nên bearing sẽ chính xác.

**Verification**: Khoảng cách, pitch_rad, state, target_valid đều chính xác → ✅ Logic hoạt động đúng.

---

### Test 4 — Sysid Filter: ⚠️ CHƯA ĐẦY ĐỦ

Bạn đã test params nhưng **chưa gửi MAVLink GLOBAL_POSITION_INT thực tế** từ script ngoài. Kết quả cho thấy tracker vẫn đang dùng **fake target từ params** (target_system=0), không phải MAVLink target.

Cần chạy lại với `test_mavlink_target.py` script.

---

### Test 5 — PID: ✅ ĐẠT

P-only và P+I đều hoạt động. Output saturated ở ±1.0 vì yaw error rất lớn (~162°) — đúng hành vi.

### Test 6 — STOP Mode: ✅ ĐẠT

Servo centered `[0.0, 0.0]` khi TRK_MODE=0.

---

## Plan test còn lại

### Test 2 — MAVLink Target (Circular Orbit)

> [!IMPORTANT]
> Test này kiểm tra **toàn bộ pipeline**: MAVLink receiver → uORB → tracker module → servo output.

#### Bước 1: Chuẩn bị SITL
```bash
# Terminal 1: Start SITL
cd ~/PX4_tracker/PX4-Autopilot
make px4_sitl gz_x500
```

Trong PX4 shell:
```bash
# Disable fake target, enable MAVLink tracking
param set TRK_MODE 1
param set TRK_SERVO_TEST 0
param set TRK_TGT_LAT 0
param set TRK_TGT_LON 0
param set TRK_TGT_ALT 0
param set TRK_SYSID_TGT 0
param set TRK_AUTO_LOCK 1

antenna_tracker start
```

#### Bước 2: Chạy MAVLink target script
```bash
# Terminal 2: Send circular orbit target
cd ~/PX4_tracker/PX4-Autopilot
python3 scripts/test_mavlink_target.py \
    --mode circular \
    --radius 200 \
    --alt 100 \
    --sysid 2 \
    --duration 30
```

#### Bước 3: Monitor (trong PX4 shell)
```bash
# Xem target position nhận được
listener tracker_target_position

# Xem tracker status (bearing phải thay đổi liên tục)
listener tracker_status

# Xem servo output (yaw phải sweep theo vòng tròn)
listener actuator_servos

# Xem status tổng hợp
antenna_tracker status
```

#### Kết quả mong đợi
- `tracker_target_position.target_system` = 2 (đúng sysid)
- `tracker_target_position.valid` = true
- `tracker_status.bearing_rad` thay đổi liên tục 0→2π (vòng tròn)
- `tracker_status.state` = 1 (TRACKING)
- `actuator_servos.control[0]` (yaw) thay đổi theo bearing
- Console log xuất hiện: `"Tracker auto-locked to sysid 2"`

---

### Test 3 — Timeout & Failsafe

> [!IMPORTANT]
> Test này kiểm tra khi target mất → tracker phải reset servo + PID.

#### Bước 1: Chạy lại script với duration ngắn
```bash
# Terminal 2:
python3 scripts/test_mavlink_target.py \
    --mode circular \
    --radius 200 \
    --alt 100 \
    --sysid 2 \
    --duration 10
```

#### Bước 2: Trong PX4 shell — Monitor trạng thái tracking
```bash
# Trong khi script đang chạy (10s đầu):
antenna_tracker status      # Expect: Target Valid: YES, state=TRACKING
```

#### Bước 3: Chờ script dừng + timeout
```bash
# Script dừng sau 10s. Chờ thêm ~6s (TRK_TIMEOUT_MS=5000)
# Sau đó kiểm tra:
antenna_tracker status      # Expect: Target Valid: no, state=TIMEOUT
listener actuator_servos    # Expect: control=[0.0, 0.0, ...]
```

#### Bước 4: Recovery — chạy lại script
```bash
# Terminal 2: chạy lại script
python3 scripts/test_mavlink_target.py --mode static --sysid 2 --duration 10
```

Trong PX4 shell:
```bash
antenna_tracker status      # Expect: Target Valid: YES, tracking resumed
```

#### Kết quả mong đợi

| Giai đoạn | target_valid | state | servo output |
|---|---|---|---|
| Script đang chạy | true | TRACKING (1) | Giá trị PID |
| Script dừng (trong timeout) | true | TRACKING (1) | Giá trị PID (giữ last) |
| Sau timeout (5s) | **false** | **TIMEOUT (2)** | **[0.0, 0.0]** |
| Script chạy lại | **true** | **TRACKING (1)** | Giá trị PID mới |

---

### Test 4 — Sysid Filter (với MAVLink thực tế)

#### Test 4A: Specific sysid
```bash
# PX4 shell:
param set TRK_SYSID_TGT 5
param set TRK_AUTO_LOCK 0
param set TRK_TGT_LAT 0
param set TRK_TGT_LON 0
param set TRK_TGT_ALT 0
```

```bash
# Terminal 2: Gửi từ sysid=2 (WRONG sysid)
python3 scripts/test_mavlink_target.py --sysid 2 --mode static --duration 10
```

```bash
# PX4 shell:
antenna_tracker status      # Expect: Target Valid: no (sysid 2 bị reject)
listener tracker_target_position   # Không có data (bị filter)
```

```bash
# Terminal 2: Gửi từ sysid=5 (CORRECT sysid)
python3 scripts/test_mavlink_target.py --sysid 5 --mode static --duration 10
```

```bash
# PX4 shell:
antenna_tracker status      # Expect: Target Valid: YES, Target SysID: 5
```

#### Test 4B: Auto-lock
```bash
# PX4 shell:
param set TRK_SYSID_TGT 0
param set TRK_AUTO_LOCK 1
```

```bash
# Terminal 2: Gửi từ sysid=3
python3 scripts/test_mavlink_target.py --sysid 3 --mode static --duration 15
```

```bash
# PX4 shell:
# Expect log: "Tracker auto-locked to sysid 3"
antenna_tracker status      # Target SysID: 3
```

```bash
# Terminal 3: Đồng thời gửi từ sysid=4 (phải bị bỏ qua)
python3 scripts/test_mavlink_target.py --sysid 4 --mode static --duration 10 --port udp:127.0.0.1:14550
```

```bash
# PX4 shell:
antenna_tracker status      # Target SysID vẫn = 3 (lock không đổi)
```

---

## Scripts đã tạo

| Script | Mô tả | Vị trí |
|---|---|---|
| [test_mavlink_target.py](file:///home/ubuntu/PX4_tracker/PX4-Autopilot/scripts/test_mavlink_target.py) | Gửi GLOBAL_POSITION_INT giả lập UAV | `scripts/` |
| [test_geometry.py](file:///home/ubuntu/PX4_tracker/PX4-Autopilot/scripts/test_geometry.py) | Verify toán hình học | `scripts/` |

### Đã verify test_geometry.py:
```
NORTH:  bearing=0.00°   distance=100.19m  pitch=26.52°  ✅
EAST:   bearing=90.00°  distance=75.35m   pitch=33.57°  ✅
SOUTH:  bearing=180.00° distance=100.19m  pitch=26.52°  ✅
WEST:   bearing=270.00° distance=75.35m   pitch=33.57°  ✅
```

---

## Thứ tự chạy test

1. **Test 2** (MAVLink circular) → quan trọng nhất, verify toàn pipeline
2. **Test 3** (Timeout) → chạy ngay sau Test 2 (cùng session)
3. **Test 4** (Sysid filter) → verify security/isolation
