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

### 1.5 Trạng thái hiện tại, cập nhật 2026-06-07

Đã có firmware antenna tracker chạy được trong PX4 v1.17.0 với các lớp chính:

- Airframe hardware `4099_antenna_tracker`.
- Airframe POSIX/SITL `4099_antenna_tracker`.
- Module `antenna_tracker` chạy 50 Hz.
- uORB `tracker_target_position` và `tracker_status`.
- Servo output qua `actuator_servos`.
- Default output mapping: `PWM_MAIN_FUNC1=201` cho yaw MAIN1, `PWM_MAIN_FUNC2=202` cho pitch MAIN2.
- SITL tracker boot bằng `PX4_SYS_AUTOSTART=4099 PX4_SIM_MODEL=none ./bin/px4`.
- POSIX SITL có GPS/baro/mag simulation để tạo `vehicle_attitude` và `vehicle_global_position`.
- QGC metadata tạo group riêng `Antenna Tracker`, dùng `@class Rover` để QGC chấp nhận airframe.

Đã xác nhận trong SITL:

```text
SYS_AUTOSTART = 4099
MAV_TYPE = 5
vehicle_attitude published
vehicle_global_position lat/lon/alt valid
antenna_tracker running
actuator_servos published
```

Lưu ý: QGC có thể hiển thị `MAV_TYPE Unknown:5`; đây là giới hạn UI, không phải lỗi firmware.

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

- [x] Tạo folder `src/modules/antenna_tracker/`
- [x] Tạo `CMakeLists.txt`
- [x] Tạo `Kconfig`
- [x] Tạo `antenna_tracker_main.cpp`
- [x] Implement command:
  - [x] `antenna_tracker start`
  - [x] `antenna_tracker stop`
  - [x] `antenna_tracker status`
- [x] Add module vào build config nếu cần
- [x] Build SITL thành công
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

- [x] Include uORB `actuator_servos`
- [x] Publish `actuator_servos.control[0]` cho yaw
- [x] Publish `actuator_servos.control[1]` cho pitch
- [x] Tạo chế độ test sweep:
  - [x] yaw: -0.5 -> 0 -> +0.5 -> 0
  - [x] pitch: -0.5 -> 0 -> +0.5 -> 0
- [x] Kiểm tra output PWM trên MAIN1/MAIN2 trong SITL
- [ ] Kiểm tra servo quay đúng chiều trên hardware
- [x] Ghi chú mapping channel yaw/pitch

### Acceptance criteria

- Servo yaw quay khi thay đổi `control[0]`
- Servo pitch quay khi thay đổi `control[1]`
- Có thể đảo chiều bằng parameter hoặc output mapping

---

## Giai đoạn 3 — Geometry + target giả

### Mục tiêu

Tính bearing/pitch/distance trước khi nhận MAVLink thật.

### TODO

- [x] Subscribe `vehicle_attitude`
- [x] Subscribe `vehicle_global_position`
- [x] Tạo parameter target giả:
  - [x] `TRK_TGT_LAT`
  - [x] `TRK_TGT_LON`
  - [x] `TRK_TGT_ALT`
- [x] Tạo tracker home fallback để test bench khi chưa có GPS/global position:
  - [x] `TRK_HOME_EN`
  - [x] `TRK_HOME_LAT`
  - [x] `TRK_HOME_LON`
  - [x] `TRK_HOME_ALT`
  - [x] `antenna_tracker set_home`
- [x] Viết `tracker_geo.hpp/.cpp`
- [x] Implement:
  - [x] longitude scale
  - [x] distance
  - [x] bearing
  - [x] pitch
  - [x] wrap yaw error
- [x] Publish debug qua `tracker_status`

### Acceptance criteria

- Đặt target giả, module tính được bearing/pitch/distance hợp lý.
- Khi không có GPS, bật `TRK_HOME_EN` với `TRK_HOME_LAT/LON/ALT` thì tracker vẫn tính được bearing/pitch để bench test.
- Khi xoay FC tracker, yaw error thay đổi đúng dấu.
- Khi đổi target giả, setpoint thay đổi đúng.

---

## Giai đoạn 4 — PID yaw/pitch

### Mục tiêu

Điều khiển servo bám theo bearing/pitch target.

### TODO

- [x] Tạo `tracker_controller.hpp/.cpp`
- [x] Implement PID đơn giản:
  - [x] P
  - [x] I với anti-windup
  - [x] D tùy chọn
  - [x] output limit `[-1, 1]`
- [x] Tạo parameter:
  - [x] `TRK_YAW_P`
  - [x] `TRK_YAW_I`
  - [x] `TRK_YAW_D`
  - [x] `TRK_PIT_P`
  - [x] `TRK_PIT_I`
  - [x] `TRK_PIT_D`
  - [x] `TRK_YAW_TRIM`
  - [x] `TRK_PIT_TRIM`
  - [x] `TRK_YAW_MIN`
  - [x] `TRK_YAW_MAX`
  - [x] `TRK_PIT_MIN`
  - [x] `TRK_PIT_MAX`
- [x] Chạy vòng lặp 50 Hz
- [x] Xuất PID ra `actuator_servos`

### Acceptance criteria

- Với target giả, antenna tự quay về hướng target.
- PID output không vượt `[-1, 1]`.
- Có thể tune P trước, sau đó thêm I/D nếu cần.

---

## Giai đoạn 5 — Airframe antenna_tracker

### Mục tiêu

Tạo airframe riêng để PX4 tự start module khi boot.

### TODO

- [x] Tạo file hardware `ROMFS/px4fmu_common/init.d/airframes/4099_antenna_tracker`
- [x] Tạo file POSIX `ROMFS/px4fmu_common/init.d-posix/airframes/4099_antenna_tracker`
- [x] Thêm metadata:
  - [x] `@name Generic Antenna Tracker`
  - [x] `@type Antenna Tracker`
  - [x] `@class Rover`
- [x] Set `VEHICLE_TYPE antenna_tracker`
- [x] Set `MAV_TYPE=5`
- [x] Set PWM output function:
  - [x] `PWM_MAIN_FUNC1=201`
  - [x] `PWM_MAIN_FUNC2=202`
- [x] Start `antenna_tracker`
- [x] Kiểm tra airframe metadata hiện trong QGC XML
- [ ] Kiểm tra chọn airframe trong QGC UI sau khi clear cache/restart
- [ ] Kiểm tra hardware boot tự chạy `antenna_tracker`
- [ ] Kiểm tra servo thật trên MAIN1/MAIN2
- [ ] Set PWM min/max/center trên hardware nếu param tồn tại

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

- [x] Tạo `msg/TrackerTargetPosition.msg`
- [x] Add message vào build
- [x] Sửa `mavlink_receiver.cpp`
- [x] Khi nhận `GLOBAL_POSITION_INT`:
  - [x] Kiểm tra sysid target
  - [x] Lưu lat/lon/alt
  - [x] Convert velocity cm/s -> m/s
  - [x] Publish `tracker_target_position`
- [x] Tạo parameter:
  - [x] `TRK_SYSID_TGT`
  - [x] `TRK_AUTO_LOCK`
  - [x] `TRK_TIMEOUT_MS`
- [x] antenna_tracker subscribe topic mới
- [x] Nếu mất target > timeout -> center servo và publish timeout status

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

- [x] Input lat/lon/alt giả
- [x] Kiểm tra distance/bearing/pitch
- [x] So với kết quả tính bằng script Python hoặc công thức độc lập

#### Test 2 — SITL module chạy độc lập

- [x] Run PX4 SITL bằng `PX4_SYS_AUTOSTART=4099 PX4_SIM_MODEL=none ./bin/px4`
- [x] Start `antenna_tracker` tự động từ airframe
- [x] Publish target giả
- [x] Check log/status

#### Test 3 — MAVLink fake target

- [x] Dùng script Python/pymavlink gửi `GLOBAL_POSITION_INT`
- [x] FC tracker nhận target
- [x] Kiểm tra `listener tracker_target_position`
- [x] Kiểm tra `listener tracker_status`

#### Test 4 — Servo output simulated

- [x] Monitor `actuator_servos`
- [x] Kiểm tra output yaw/pitch thay đổi theo target

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

## 9. Rủi ro kỹ thuật

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

## 10. Việc cần làm ngay bây giờ

Phần kế hoạch ban đầu bên dưới đã hoàn thành phần lớn trong SITL:

```text
1. Module skeleton antenna_tracker: done
2. Build SITL: done
3. Test command start/status/stop: done
4. Publish actuator_servos test: done
5. Airframe antenna_tracker: done
6. SITL sensor fusion simulation: done
7. MAVLink fake target: done
8. Hardware target/build/servo thật: pending
```

Ưu tiên tiếp theo là test hardware an toàn trước khi bay thật.

---

## 11. Checklist cho lần update/debug tiếp theo

### 11.1 Khi boot SITL tracker

Lệnh đúng:

```bash
cd build/px4_sitl_default
PX4_SYS_AUTOSTART=4099 PX4_SIM_MODEL=none ./bin/px4
```

Không dùng `make px4_sitl gz_x500` để test airframe 4099 vì `gz_x500` ép `SYS_AUTOSTART=4001`.

Kiểm tra trong PX4 shell:

```sh
param show SYS_AUTOSTART
param show MAV_TYPE
param show SENS_EN_GPSSIM
param show SENS_EN_BAROSIM
param show SENS_EN_MAGSIM
antenna_tracker status
listener vehicle_attitude
listener vehicle_global_position
listener tracker_target_position
listener tracker_status
listener actuator_servos
listener actuator_outputs
```

Kỳ vọng:

```text
SYS_AUTOSTART = 4099
MAV_TYPE = 5
vehicle_attitude published
vehicle_global_position lat/lon/alt valid
antenna_tracker running
```

### 11.2 Khi test hardware Pixhawk 6C

Việc cần calibrate:

- [ ] Accelerometer.
- [ ] Gyroscope.
- [ ] Compass/magnetometer.
- [ ] Level horizon.
- [ ] GPS fix ngoài trời.
- [ ] Power module nếu dùng.
- [ ] Servo yaw MAIN1.
- [ ] Servo pitch MAIN2.

Kiểm tra sau khi nạp firmware:

```sh
param show SYS_AUTOSTART
param show MAV_TYPE
param show PWM_MAIN_FUNC1
param show PWM_MAIN_FUNC2
listener vehicle_attitude
listener vehicle_global_position
antenna_tracker status
```

Kỳ vọng:

```text
SYS_AUTOSTART = 4099
MAV_TYPE = 5
PWM_MAIN_FUNC1 = 201
PWM_MAIN_FUNC2 = 202
vehicle_attitude published
vehicle_global_position valid
```

### 11.3 Khi servo không chạy

Debug theo thứ tự:

```sh
param set TRK_SERVO_TEST 1
param set TRK_MODE 0
listener actuator_servos
listener actuator_outputs
param show PWM_MAIN_FUNC1
param show PWM_MAIN_FUNC2
commander arm -f
listener actuator_outputs
```

Diễn giải:

- Nếu `actuator_servos.control[0/1]` đổi nhưng `actuator_outputs` không đổi: kiểm tra arming/disarmed và output mapping.
- Nếu `actuator_outputs[0/1]` đổi nhưng servo không quay: kiểm tra dây, rail power, servo, MAIN/AUX đúng cổng.
- Nếu servo quay ngược: chỉnh reverse/direction hoặc đổi dấu output sau khi xác nhận cơ khí.

### 11.4 Khi tracking không đúng hướng

Kiểm tra:

```sh
listener vehicle_attitude
listener vehicle_global_position
listener tracker_target_position
listener tracker_status
```

Nguyên nhân thường gặp:

- Compass/yaw sai hoặc chưa calibrate.
- GPS tracker không valid.
- Target sysid sai hoặc chưa auto-lock.
- Altitude reference giữa tracker và UAV không thống nhất.
- Servo yaw/pitch bị đảo chiều.
