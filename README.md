# ESP32-S3 ICM-42688-P 9-axis AHRS

Standalone ESP-IDF 5.5.1 project for an ESP32-S3 with **ICM-42688-P**, BMM350 and a
0.91 inch 128x32 I2C SSD1306 OLED. It is a port of the verified MPU6050 project
(`../ESP32S3-9Axis-AHRS`): the AHRS, magnetometer, display and serial format are
unchanged; only the IMU driver, its mounting transform and the project name were
replaced.

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

If pitch/roll signs are inverted or swapped, fix `imu_to_body()` (currently a no-op)
rather than touching the AHRS math.

## Request magnetometer recalibration

The `RECAL_MAG` serial command and the PC viewer button have the same known limitation
as the MPU6050 project: firmware reads the primary UART0 console while the board is
wired to the native USB-Serial-JTAG port. Use `CONFIG_APP_MAG_FORCE_RECAL=y` for one
flash if a recalibration is needed, then turn it back off.

## Serial output

Same 18-column `ATT` format as the MPU6050 project:

`ATT,ms,roll,pitch,yaw,mag_present,mag_calibrated,accel_x_mg,accel_y_mg,accel_z_mg,mag_fresh,mag_heading_valid,mag_heading_deg,cal_mag_x,cal_mag_y,cal_mag_z,raw_mapped_mag_x,raw_mapped_mag_y,raw_mapped_mag_z`

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
