# TODO.md — PX4 Antenna Tracker Firmware

## 1. Thông tin đã chốt

### 1.1 Hướng kiến trúc

- Làm **PX4-native antenna tracker firmware**.
- Không dùng Gimbal v2 làm hướng chính.
- FC tracker sẽ chạy firmware PX4 custom riêng.
- Antenna tracker là một airframe/module riêng trong PX4.
- Module `antenna_tracker` chịu trách nhiệm:
  - nhận vị trí UAV mục tiêu,
  - đọc attitude tracker,
  - tính bearing/pitch/distance,
  - PID yaw/pitch,
  - xuất servo qua `actuator_servos`.

---

### 1.2 Firmware base

```text
Repo: https://github.com/PX4/PX4-Autopilot
Tag: v1.17.0
Branch custom đề xuất: antenna_tracker_v1.17
Ngôn ngữ: C++
```


---

### 1.3 Kiến trúc target

```text
UAV PX4
  ↓ MAVLink GLOBAL_POSITION_INT
FC Tracker PX4 custom
  ↓ mavlink_receiver
uORB tracker_target_position
  ↓ antenna_tracker module
bearing/pitch/distance + PID
  ↓ actuator_servos
PWM output
  ↓
Servo yaw/pitch
```

---

### 1.4 Các công thức chính

#### Distance

```math
distance = \sqrt{dlat^2 + dlng_{scaled}^2} \times LOCATION\_SCALING\_FACTOR
```

#### Bearing

```math
bearing = \frac{\pi}{2} + atan2(-off_y, off_x)
```

#### Pitch

```math
pitch = atan2(\Delta altitude, distance)
```

---

## 2. File/thư mục dự kiến

```text
PX4-Autopilot/
├── msg/
│   ├── TrackerTargetPosition.msg
│   └── TrackerStatus.msg
│
├── src/modules/antenna_tracker/
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── antenna_tracker_main.cpp
│   ├── antenna_tracker.hpp
│   ├── tracker_geo.cpp
│   ├── tracker_geo.hpp
│   ├── tracker_controller.cpp
│   ├── tracker_controller.hpp
│   └── tracker_params.c
│
├── src/modules/mavlink/
│   ├── mavlink_receiver.cpp
│   └── mavlink_receiver.h
│
└── ROMFS/px4fmu_common/init.d/airframes/
    └── 4099_antenna_tracker
```

---

## 3. Giai đoạn triển khai

## Giai đoạn 1 — Module skeleton

### Mục tiêu

Tạo module `antenna_tracker` rỗng, build được trong PX4 v1.17.0.

### TODO

- [ ] Tạo folder `src/modules/antenna_tracker/`
- [ ] Tạo `CMakeLists.txt`
- [ ] Tạo `Kconfig`
- [ ] Tạo `antenna_tracker_main.cpp`
- [ ] Implement command:
  - [ ] `antenna_tracker start`
  - [ ] `antenna_tracker stop`
  - [ ] `antenna_tracker status`
- [ ] Add module vào build config nếu cần
- [ ] Build SITL thành công
- [ ] Build firmware hardware target thành công

### Acceptance criteria

```bash
antenna_tracker start
antenna_tracker status
antenna_tracker stop
```

chạy được trong PX4 shell.

---

## Giai đoạn 2 — Test servo output

### Mục tiêu

Kiểm tra pipeline:

```text
antenna_tracker module → actuator_servos → PWM output → servo
```

### TODO

- [ ] Include uORB `actuator_servos`
- [ ] Publish `actuator_servos.control[0]` cho yaw
- [ ] Publish `actuator_servos.control[1]` cho pitch
- [ ] Tạo chế độ test sweep:
  - [ ] yaw: -1 → 0 → +1
  - [ ] pitch: -1 → 0 → +1
- [ ] Kiểm tra output PWM trên MAIN/AUX
- [ ] Kiểm tra servo quay đúng chiều
- [ ] Ghi chú mapping channel yaw/pitch

### Acceptance criteria

- Servo yaw quay khi thay đổi `control[0]`
- Servo pitch quay khi thay đổi `control[1]`
- Có thể đảo chiều bằng parameter hoặc output mapping

---

## Giai đoạn 3 — Geometry + target giả

### Mục tiêu

Tính bearing/pitch/distance trước khi nhận MAVLink thật.

### TODO

- [ ] Subscribe `vehicle_attitude`
- [ ] Subscribe `vehicle_global_position`
- [ ] Tạo parameter target giả:
  - [ ] `TRK_TGT_LAT`
  - [ ] `TRK_TGT_LON`
  - [ ] `TRK_TGT_ALT`
- [ ] Viết `tracker_geo.hpp/.cpp`
- [ ] Implement:
  - [ ] longitude scale
  - [ ] distance
  - [ ] bearing
  - [ ] pitch
  - [ ] wrap yaw error
- [ ] Publish debug qua `tracker_status`

### Acceptance criteria

- Đặt target giả, module tính được bearing/pitch/distance hợp lý.
- Khi xoay FC tracker, yaw error thay đổi đúng dấu.
- Khi đổi target giả, setpoint thay đổi đúng.

---

## Giai đoạn 4 — PID yaw/pitch

### Mục tiêu

Điều khiển servo bám theo bearing/pitch target.

### TODO

- [ ] Tạo `tracker_controller.hpp/.cpp`
- [ ] Implement PID đơn giản:
  - [ ] P
  - [ ] I với anti-windup
  - [ ] D tùy chọn
  - [ ] output limit `[-1, 1]`
- [ ] Tạo parameter:
  - [ ] `TRK_YAW_P`
  - [ ] `TRK_YAW_I`
  - [ ] `TRK_YAW_D`
  - [ ] `TRK_PIT_P`
  - [ ] `TRK_PIT_I`
  - [ ] `TRK_PIT_D`
  - [ ] `TRK_YAW_TRIM`
  - [ ] `TRK_PIT_TRIM`
  - [ ] `TRK_YAW_MIN`
  - [ ] `TRK_YAW_MAX`
  - [ ] `TRK_PIT_MIN`
  - [ ] `TRK_PIT_MAX`
- [ ] Chạy vòng lặp 50 Hz
- [ ] Xuất PID ra `actuator_servos`

### Acceptance criteria

- Với target giả, antenna tự quay về hướng target.
- PID output không vượt `[-1, 1]`.
- Có thể tune P trước, sau đó thêm I/D nếu cần.

---

## Giai đoạn 5 — Airframe antenna_tracker

### Mục tiêu

Tạo airframe riêng để PX4 tự start module khi boot.

### TODO

- [ ] Tạo file `4099_antenna_tracker`
- [ ] Thêm metadata:
  - [ ] `@name Generic Antenna Tracker`
  - [ ] `@type Antenna Tracker`
  - [ ] `@class Tracker`
- [ ] Start sensor/estimator tối thiểu
- [ ] Set MAVLink link
- [ ] Set PWM min/max/center
- [ ] Start `antenna_tracker`
- [ ] Kiểm tra airframe hiện trong QGC

### Acceptance criteria

- Flash firmware.
- Chọn được airframe antenna tracker.
- Boot lên tự chạy `antenna_tracker`.
- Servo ở trạng thái an toàn khi chưa có target.

---

## Giai đoạn 6 — MAVLink target receiver

### Mục tiêu

Nhận vị trí UAV thật từ MAVLink.

### TODO

- [ ] Tạo `msg/TrackerTargetPosition.msg`
- [ ] Add message vào build
- [ ] Sửa `mavlink_receiver.cpp`
- [ ] Khi nhận `GLOBAL_POSITION_INT`:
  - [ ] Kiểm tra sysid target
  - [ ] Lưu lat/lon/alt
  - [ ] Convert velocity cm/s → m/s
  - [ ] Publish `tracker_target_position`
- [ ] Tạo parameter:
  - [ ] `TRK_SYSID_TARGET`
  - [ ] `TRK_AUTO_LOCK`
  - [ ] `TRK_TIMEOUT_MS`
- [ ] antenna_tracker subscribe topic mới
- [ ] Nếu mất target > timeout → STOP hoặc HOLD

### Acceptance criteria

- UAV gửi `GLOBAL_POSITION_INT`.
- FC tracker nhận và publish ra `tracker_target_position`.
- Module chuyển từ target giả sang target thật.

---

## Giai đoạn 7 — Mô phỏng/SITL

### Mục tiêu

Test firmware trước khi đưa lên phần cứng.

### Kịch bản đề xuất

#### Test 1 — Unit test geometry

- Input lat/lon/alt giả
- Kiểm tra distance/bearing/pitch
- So với kết quả tính bằng script Python hoặc công thức độc lập

#### Test 2 — SITL module chạy độc lập

- Run PX4 SITL
- Start `antenna_tracker`
- Publish target giả
- Check log/status

#### Test 3 — MAVLink fake target

- Dùng script Python/MAVSDK/pymavlink gửi `GLOBAL_POSITION_INT`
- FC tracker nhận target
- Kiểm tra `listener tracker_target_position`
- Kiểm tra `listener tracker_status`

#### Test 4 — Servo output simulated

- Monitor `actuator_servos`
- Kiểm tra output yaw/pitch thay đổi theo target

#### Test 5 — Hardware-in-loop nhẹ

- FC thật + servo thật
- UAV target giả bằng script MAVLink
- Không cần bay thật
- Kiểm tra servo bám hướng

---

## Giai đoạn 8 — Hoàn thiện sau bản đầu

### TODO mở rộng

- [ ] AUTO mode hoàn chỉnh
- [ ] STOP mode
- [ ] SCAN mode tìm lại target
- [ ] MANUAL mode nếu có RC
- [ ] SERVOTEST command
- [ ] Auto-lock sysid giống ArduPilot
- [ ] Baro/GPS altitude source option
- [ ] Dead reckoning khi mất gói ngắn hạn
- [ ] Logger/debug topic đầy đủ
- [ ] Tham số reverse servo
- [ ] Hỗ trợ continuous rotation servo nếu cần

---

## 4. Rủi ro kỹ thuật

### Rủi ro 1 — PX4 output bị module khác chiếm

Cần đảm bảo airframe antenna tracker không start multicopter/fixed-wing controller không cần thiết.

### Rủi ro 2 — MAVLink receiver publish sai target

Cần lọc đúng sysid UAV mục tiêu. Không lấy nhầm message từ GCS hoặc tracker.

### Rủi ro 3 — Servo mapping sai

Cần test output trước khi chạy PID thật. Sai chiều servo có thể làm antenna quay ngược.

### Rủi ro 4 — Compass/yaw không ổn định

Antenna tracker dùng yaw từ AHRS. Nếu compass nhiễu, yaw feedback sai và tracking sẽ sai.

### Rủi ro 5 — Pitch reference không thống nhất

Cần xác định rõ pitch servo đo theo thân tracker, earth frame hay body frame.

---

## 5. Việc cần làm ngay bây giờ

Ưu tiên thấp rủi ro nhất:

```text
1. Clone PX4 v1.17.0 (da clone)
2. Tạo branch antenna_tracker_v1.17
3. Tạo module skeleton antenna_tracker
4. Build được SITL
5. Build được firmware board thật
6. Test command start/status/stop
7. Publish actuator_servos test
8. Tạo airframe antenna_tracker
```

Chỉ sau khi các bước này chạy ổn mới sửa `mavlink_receiver`.
