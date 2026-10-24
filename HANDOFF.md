# Handoff: ESP32-S3 ICM-42688-P 9-axis AHRS

This project is a port of the verified MPU6050 build in `../ESP32S3-9Axis-AHRS`.
Everything except the IMU driver, its mounting transform and the project name is
unchanged: AHRS math, BMM350 handling, axis detection, OLED renderer, serial format and
the PC viewer all come from that project (see its HANDOFF for the full background and
the [ATTITUDE-FUSION.md](./ATTITUDE-FUSION.md) document for the method).

## What changed relative to the MPU6050 project

- `main/mpu6050.c/.h` replaced by `main/icm42688p.c/.h` (I2C, Bank 0 register map).
- `imu_to_body()` is now an identity transform; the old 90-degree rotation is gone.
- Root `CMakeLists.txt` project name is `esp32s3_icm42688p_ahrs`, so the app binary is
  `build/esp32s3_icm42688p_ahrs.bin`.
- `sdkconfig` keeps the verified settings: ESP32-S3, 16 MB, 0.91" 128x32 OLED
  (`CONFIG_APP_OLED_HO_32`), BMM350 enabled, axis map/calibration left in NVS.

## Axis derivation (why identity is correct)

The MPU6050 build used `body = (-sy, sx, sz)`. Inverting that gives the old module's
printed directions: `+X` left, `+Y` back, `+Z` up.

The ICM-42688-P module is mounted with `+X` forward, `+Y` left, `+Z` up, which is
exactly the board body frame. Going from the old directions to the new ones is a
90-degree clockwise rotation about `+Z`, matching the user's description ("the MPU6050
module was rotated 90 degrees clockwise").

The magnetometer did not move, so its saved map `idx=19`
(`body = (+my, +mx, -mz)`) is unchanged. Note that this map flips the BMM350's raw
`+Z`, which is why the magnetometer's chip axes and the IMU axes look mirrored while the
mapped frames agree.

## ICM-42688-P driver notes

- `WHO_AM_I` (`0x75`) must read `0x47`; I2C address is `0x68` (AP_AD0 low) or `0x69`.
- Init sequence: soft reset (`0x11 = 0x01`, wait 10 ms), poll `WHO_AM_I`,
  `GYRO_CONFIG0 = 0x47` (+-500 dps, 100 Hz), `ACCEL_CONFIG0 = 0x67` (+-2 g, 100 Hz),
  `PWR_MGMT0 = 0x0F` (both low-noise), wait 60 ms for the gyro to settle, then read back
  and log the three registers.
- Data: one 14-byte burst from `0x1D` = temperature, accel XYZ, gyro XYZ (big endian).
  `raw[]` keeps the old MPU6050 order `ax ay az temp gx gy gz`.
- Gyro bias calibration adds a 100 ms settle plus no sample discarding; verify whether
  this removes the `sigma ~11 deg/s` transient seen in the MPU6050 build.

## Verification status

Not yet verified on hardware. The magnetometer side is expected to work immediately
because the same board and the same NVS state are reused. Check in this order:

1. Boot log: `ICM42688P 就绪` with `WHO_AM_I=0x47 PWR=0x0F GYRO=0x47 ACCEL=0x67`,
   then `陀螺零偏校准完成` with a sane bias and noise.
2. Static: `ATT` shows `mag_present=1`, `mag_calibrated=1`; level board gives near-zero
   roll/pitch; yaw sigma < 0.5 deg.
3. Tilt 30 degrees: yaw change < 2 deg.
4. Edge-aligned 180 degree turn: yaw change 180 +- 5 deg.
5. If roll/pitch signs or axes are wrong, fix `imu_to_body()` only.

## Known issues carried over

- `RECAL_MAG` now works over the native USB port: the primary console is
  USB-Serial-JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, secondary NONE), so the
  command task receives viewer input. Application logs are USB-only; UART0 no longer
  prints app logs. The command task also logs `serial command received: ...` for any
  line, which makes the path testable without clearing the calibration (send `PING`).
- Magnetic accuracy depends on the environment; keep the board away from laptops,
  speakers and metal when calibrating and when using it.
- Min-max magnetometer calibration does not model cross-axis soft iron; the calibrated
  field magnitude can vary with position.

## Next steps

1. Build and flash, then run the checklist above and record the numbers in this file.
2. Compare ICM-42688-P static noise and drift against the MPU6050 baseline
   (sigma 0.13-0.38 deg, drift -0.19 deg/min).
3. If the IMU axes turn out differently in practice, correct `imu_to_body()` with a
   proper rotation (determinant +1) and re-run the tilt test.

## ICM-42688-P migration and bring-up (2026-10-24)

Hardware: the MPU6050 module was replaced by an ICM-42688-P module on the same board
(same ESP32-S3, same BMM350 and OLED). The module `CS` is tied high (I2C mode) and
`AP_AD0` is tied high, so its 7-bit address is **0x69**. The magnetometer and its axis
map (`idx=19`) did not move, and the stored magnetic calibration is reused.

Board wiring note (important): this module does **not** support I2C repeated-start
reads. `i2c_master_transmit_receive()` returned NACK on every register read while the
plain address probe still ACKed. Fix: the driver uses split transactions (write the
register address, then a separate receive) for both `icm42688p_read_reg()` and the
14-byte data burst. `ICM42688P_I2C_SPEED_HZ` is set to 100 kHz.

Init sequence verified on hardware:
`ICM42688P ready: addr=0x69 WHO_AM_I=0x47 PWR=0x0F GYRO=0x47 ACCEL=0x67` with
`+-2 g / +-500 dps / 100 Hz`. `main.c` tries 0x68 first and falls back to 0x69.

Axis mapping: module `+X` forward, `+Y` left, `+Z` up equals the body frame, so
`imu_to_body()` is a no-op. Verified by the level reading: accel `(-18, -23, 1018) mg`,
i.e. `+Z` sees +1 g and the magnitude is 1019 mg.

Static measurement (30 s, board level and still):

- gyro bias `[-0.429, -0.062, -0.044] deg/s`, noise `0.026 deg/s`; the 100 ms settle
  before sampling removed the startup transient that read 11.4 deg/s on the MPU6050;
- yaw sigma `0.08 deg`, drift `0.5 deg/min`, yaw vs magnetic heading `0.02 +- 0.17 deg`;
- calibrated field magnitude `57.4 uT` against the stored radius `45.8 uT`
  (position dependent, same behaviour as the MPU6050 build).

Pending: tilt (~30 deg) and edge-aligned 180 deg dynamic checks, to be recorded after
the next measurement session.
