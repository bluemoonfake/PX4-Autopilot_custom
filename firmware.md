# PX4 Firmware Recovery Checklist for MicoAir H743 on Ubuntu

This checklist is for the case where:

- the board is a `MicoAir H743`
- PX4 upload can see bootloader `board id 1166`
- USB shows `ArduPilot_MicoAir743-BL`
- the board disappears after a few seconds
- QGroundControl cannot connect after reboot

The likely root cause is a bootloader/app layout mismatch. The recovery path is:

1. enter STM32 DFU mode
2. flash the PX4 bootloader for `boards/micoair/h743`
3. reboot
4. upload PX4 firmware again
5. verify USB enumeration and QGroundControl connection

## 1. Files used in this repo

PX4 bootloader:

```bash
boards/micoair/h743/extras/micoair_h743_bootloader.bin
```

PX4 antenna tracker firmware:

```bash
build/micoair_h743_antenna_tracker/micoair_h743_antenna_tracker.px4
```

Build command if firmware is not present yet:

```bash
make micoair_h743_antenna_tracker
```

## 2. Install required Ubuntu tools

Check that `dfu-util` exists:

```bash
dfu-util --version
```

If missing:

```bash
sudo apt update
sudo apt install dfu-util
```

## 3. Put the board into STM32 DFU mode

Disconnect the board from USB.

Then use this sequence:

1. hold the `BOOT` button
2. plug in USB, or hold `BOOT` and tap `RESET`
3. release `RESET` if used
4. release `BOOT`

Expected USB identity in DFU mode:

- `Manufacturer: STMicroelectronics`
- `Product: DFU in FS Mode`

Verify with:

```bash
dfu-util -l
```

Expected result includes something similar to:

```text
Found DFU: [0483:df11]
```

If DFU does not appear:

- try another USB data cable
- try another USB port
- disconnect all servos and peripherals
- power only from USB during recovery

## 4. Flash the PX4 bootloader

Run from the repo root:

```bash
dfu-util -a 0 -s 0x08000000:leave -D boards/micoair/h743/extras/micoair_h743_bootloader.bin
```

Expected outcome:

- erase/program completes successfully
- board leaves DFU mode automatically

If `dfu-util` reports permission problems, retry with:

```bash
sudo dfu-util -a 0 -s 0x08000000:leave -D boards/micoair/h743/extras/micoair_h743_bootloader.bin
```

## 5. Verify the new bootloader appears

Unplug and reconnect the board normally.

Then check:

```bash
ls /dev/serial/by-id
ls /dev/ttyACM*
dmesg | tail -n 50
```

Expected behavior:

- a USB serial device appears again
- bootloader should enumerate first
- the old `ArduPilot_MicoAir743-BL` identity should no longer be the reference point for recovery

Also test PX4 uploader detection:

```bash
make micoair_h743_antenna_tracker upload
```

The important check is that the uploader still finds:

- `board id: 1166`

If bootloader replacement succeeded, it should no longer behave like the previous ArduPilot bootloader with the smaller app window.

## 6. Build and upload PX4 antenna tracker firmware

Build:

```bash
make micoair_h743_antenna_tracker
```

Upload:

```bash
make micoair_h743_antenna_tracker upload
```

Expected outcome:

- bootloader is found
- erase/program/verify succeed
- board reboots into app

## 7. Verify the app boots after upload

Open a terminal and monitor kernel messages:

```bash
dmesg -w
```

Reconnect the board and check:

```bash
ls /dev/serial/by-id
ls /dev/ttyACM*
```

Expected good behavior:

1. board may appear briefly as bootloader
2. bootloader disconnects
3. app enumerates again as a running PX4 USB ACM device
4. `/dev/ttyACM0` remains present

Bad behavior:

1. board appears only as bootloader
2. bootloader disconnects after about 5 seconds
3. no new PX4 app USB device appears

If bad behavior continues after bootloader replacement, the app is still failing very early in boot and needs firmware-side debugging.

## 8. Verify in QGroundControl

After the app stays present on USB:

1. open `QGroundControl`
2. reconnect the board
3. wait for auto-connect

Expected result:

- QGroundControl sees the board
- parameters can be read
- firmware no longer disappears immediately

## 9. Optional validation with default PX4 target

After replacing the bootloader, test whether the default target still reports image-size mismatch:

```bash
make micoair_h743_default upload
```

Before bootloader recovery, this failed with:

```text
Error: Firmware image is too large for this board
```

If bootloader layout is now correct, this error should no longer be caused by the old smaller boot window.

## 10. Quick decision table

If `dfu-util -l` does not show `0483:df11`:

- DFU mode was not entered

If bootloader upload succeeds but board still only shows bootloader and never app:

- firmware app is still crashing or not booting

If bootloader upload succeeds and the board stays visible as `/dev/ttyACM0` after reboot:

- USB app boot is working again

If QGroundControl still does not connect but `/dev/ttyACM0` stays present:

- PX4 is likely running and the issue is MAVLink/USB startup, not flashing

## 11. Commands summary

Enter DFU and verify:

```bash
dfu-util -l
```

Flash PX4 bootloader:

```bash
dfu-util -a 0 -s 0x08000000:leave -D boards/micoair/h743/extras/micoair_h743_bootloader.bin
```

Build tracker firmware:

```bash
make micoair_h743_antenna_tracker
```

Upload tracker firmware:

```bash
make micoair_h743_antenna_tracker upload
```

Watch USB behavior:

```bash
dmesg -w
```

Check USB serial nodes:

```bash
ls /dev/serial/by-id
ls /dev/ttyACM*
```
