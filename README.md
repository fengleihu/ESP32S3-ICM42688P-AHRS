# ESP32-S3 ICM-42688-P 9-axis AHRS

Standalone ESP-IDF 5.5.1 project for an ESP32-S3 with **ICM-42688-P**, BMM350 and a
0.91 inch 128x32 I2C SSD1306 OLED. It is a port of the verified MPU6050 project
(`../ESP32S3-9Axis-AHRS`): the AHRS, display and serial format are unchanged; the IMU
driver, its mounting transform, the project name and the magnetometer calibration
method (now an ellipsoid fit) were replaced.

The fusion method is explained in [ATTITUDE-FUSION.md](./ATTITUDE-FUSION.md); current
work notes and open items are in [HANDOFF.md](./HANDOFF.md).

## Hardware and wiring

| Device | Bus / address |
|---|---|
| ICM-42688-P | I2C `0x68` (AP_AD0 low) or `0x69` |
| BMM350 | I2C `0x14` (driver also probes `0x15`) |
| SSD1306 0.91" 128x32 | I2C `0x3C` (also `0x3D`) |

Shared bus: SDA GPIO1, SCL GPIO2, 400 kHz, 3V3.

**IMU axes:** the module's printed `+X` points forward (photo top), `+Y` left, `+Z` up,
so the module frame equals the body frame and `imu_to_body()` is an identity
transform. The magnetometer is a separate chip: the saved BMM350 axis map `idx=19`
maps `body = (+my, +mx, -mz)`, i.e. it flips the magnetometer's raw `+Z`; that map is
unchanged and still valid because the magnetometer did not move.

## Display

0.91 inch 128x32 panel: a wireframe attitude cube (fixed 30 deg azimuth / 20 deg
elevation view, nose spike along `+X`) on the left and three 1x `R/P/Y` lines on the
right; `MAG CAL` / `MAG AXIS` screens stay text-only. Same renderer as the MPU6050
project (`main/oled_ui.c`).

## Build and flash

Build with the ESP-IDF extension or `idf.py build` in this folder. The application
binary is now `build/esp32s3_icm42688p_ahrs.bin`.

Flashing without `idf.py`:

```
python -m esptool --chip esp32s3 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 16MB --flash_freq 80m 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x10000 build\esp32s3_icm42688p_ahrs.bin
```

Keep the board still for the one-second gyro bias window after reset.

## First-run checks (new IMU)

The magnetometer calibration and axis map are reused from the same board's NVS
namespace `att9axis`, so a fresh axis detection should NOT run. What does need checking:

1. Boot log shows `ICM42688P 就绪：... WHO_AM_I=0x47` and the expected
   `PWR=0x0F GYRO=0x47 ACCEL=0x67`.
2. Level the board: `pitch` near 0 and `roll` near 0; tilting forward should make pitch
   positive (nose down positive per the AHRS convention).
3. Static yaw sigma and drift comparable to the MPU6050 build (sigma < 0.5 deg).
4. Tilt 30 degrees: yaw should move less than a couple of degrees.
5. Edge-aligned 180 degree turn: yaw should change 180 +- 5 degrees.

Steps 3-5 are automated: `tools/att_report.py record` / `analyze` finds the steady
plateaus in a recording and prints PASS/FAIL with the measured numbers (see
[HANDOFF.md](./HANDOFF.md#four-criteria-measurement-still-to-run-on-hardware)).

If pitch/roll signs are inverted or swapped, fix `imu_to_body()` (currently a no-op)
rather than touching the AHRS math.

## Magnetometer calibration

`mag_cal_process()` fits an **ellipsoid** to the samples collected during the 15 s
rotation (`main/mag_fit.c`): hard-iron offset plus a full 3x3 soft-iron matrix, so
cross-axis coupling is modelled too. If the fit is rejected (too few samples, a
degenerate/partial rotation, an implausible result) the firmware falls back to the old
min-max method and logs a warning. Sample coverage is checked independently by the
min/max span ratio (`quality`, gate 0.4); a fit on a partial rotation can look perfect
while the centre is tens of uT off.

Typical log lines:

```
I (1234) mag: 校准完成（椭球拟合）：样本 375（拟合 372，剔除 3） offset=[24.6, -39.8, 11.9] µT 半径=57.4 µT 残差=1.20% 覆盖度=0.88
I (1300) att9axis: Mag calibration OK: samples=375 method=椭球拟合 quality=0.88 radius=57.4 uT residual=1.20%
```

An existing `MAG1` (min-max) NVS blob is converted on load so an upgrade does not
clear the calibration, but the boot log recommends running `RECAL_MAG` once.

## Request magnetometer recalibration

The `RECAL_MAG` serial command works over the native USB port. `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`
makes USB-Serial-JTAG the primary console, so the firmware command task receives typed input.
Type `RECAL_MAG` (or use the viewer button) and the firmware stores a one-shot request,
restarts, clears only the magnetic calibration, and asks for the 15-second rotation.
`CONFIG_APP_MAG_FORCE_RECAL` remains available as a build-time override. Application logs
now go to the USB console only; UART0 no longer prints application logs.

`MARK <label>` prints `MARK,<ms>,<label>` immediately, which is how the analysis tool
segments a measurement session. The command buffer is 64 bytes, so labels are not
truncated.

## Tools

| Tool | What it does |
|---|---|
| `tools/att_report.py record` | captures the serial log (`--port COM3 --seconds 90 --out logs/x.log`; the board enumerates as COM3 on this machine); typed lines are forwarded to the board, so `MARK level` etc. can be sent while recording |
| `tools/att_report.py analyze` | finds steady plateaus, prints static/tilt/180 deg metrics and PASS/FAIL |
| `tools/host_test/mag_fit_test.c` | host-side regression test for the ellipsoid solver (`gcc -O2 -I main tools/host_test/mag_fit_test.c main/mag_fit.c -lm -o mag_fit_test`) |
| `tools/host_test/make_test_log.py` | builds a synthetic ATT log with known answers (`--bad` for a failing one) to regression-test the analyzer |

`tools/att_report.py record` needs `pyserial`; `analyze` is pure stdlib.

## Serial output

`ATT` line every 100 ms. The first 19 tokens are the same as the MPU6050 project;
tokens 20-22 are new (gyro, for the measurement tool):

```
ATT,ms,roll,pitch,yaw,mag_present,mag_calibrated,
    accel_x_mg,accel_y_mg,accel_z_mg,mag_fresh,mag_heading_valid,mag_heading_deg,
    cal_mag_x,cal_mag_y,cal_mag_z,raw_mapped_mag_x,raw_mapped_mag_y,raw_mapped_mag_z,
    gyro_x_dps,gyro_y_dps,gyro_z_dps
```

The gyro columns exist so `tools/att_report.py` can compare a heading change against an
independent inertial reference: a magnetic heading that moves while the gyro integral
stays at zero is a magnetism problem, not a rotation of the board.

`pc_viewer` parses the first columns by index and ignores the rest.

## PC attitude viewer

`pc_viewer/` is the same browser viewer (Chrome/Edge Web Serial, no backend). See
[pc_viewer/README.md](./pc_viewer/README.md).

## Driver notes

`main/icm42688p.c` uses the ICM-42688-P Bank 0 register map:

- `0x11 DEVICE_CONFIG` soft reset, `0x75 WHO_AM_I` = `0x47`;
- `0x1D..0x2A` one 14-byte burst = temperature + accel XYZ + gyro XYZ;
- `0x4F GYRO_CONFIG0` = `+-500 dps` + `100 Hz` (`0x47`),
  `0x50 ACCEL_CONFIG0` = `+-2 g` + `100 Hz` (`0x67`);
- `0x4E PWR_MGMT0` = `0x0F` (gyro low-noise + accel low-noise), then a 60 ms wait for
  the gyro to settle.

Gyro bias calibration adds a 100 ms settle before sampling, avoiding the startup
transient seen in the MPU6050 build.

## I2C bus recovery

The board's I2C wiring is dupont leads, and moving the board can disturb SDA/SCL. The
ESP-IDF i2c driver then stops its state machine after a failed bus clear, and every
later transaction returns `ESP_ERR_INVALID_STATE` — IMU, magnetometer and OLED all stay
dead until a reboot. `app_i2c_bus_recover()` (`main/att_i2c_bus.c`) resets the bus
(9 SCL clocks + FSM reset); the main loop calls it after 5 consecutive read failures and
then every 0.5 s until reads succeed, logging `I2C 总线已复位`. If it keeps failing the
log says the lines are really held low, which is a wiring/power problem.
