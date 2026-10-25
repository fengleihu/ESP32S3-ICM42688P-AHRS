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
- The old min-max calibration modelled no cross-axis soft iron, so the calibrated
  field magnitude varied with position (57.4 uT measured against a stored radius of
  45.8 uT). **Fixed 2026-10-25** by the ellipsoid fit below; an old stored min-max
  calibration is still loadable (converted to a diagonal `soft` matrix) but should be
  replaced with a fresh `RECAL_MAG`.

## Next steps

1. Build and flash, then run the four-criteria session below in a clean location and
   record the numbers in the results table.
2. Compare ICM-42688-P static noise and drift against the MPU6050 baseline
   (sigma 0.13-0.38 deg, drift -0.19 deg/min).
3. If the IMU axes turn out differently in practice, correct `imu_to_body()` with a
   proper rotation (determinant +1) and re-run the tilt test.

## Magnetometer calibration: ellipsoid fit (2026-10-25)

### Why the change

Min-max calibration only extracts a per-axis midpoint and half-span, i.e. it models
hard iron plus **axis-aligned** soft iron. BMM350 sits next to the IMU, the OLED and
the regulator, so the real soft-iron term has off-diagonal (cross-axis) components.
The symptom on hardware was that the calibrated field magnitude moved between 45 and
57 uT depending on where the board was, and that a fresh 15 s rotation gave a
different radius each time.

### What it does now

- New `main/mag_fit.c` + `main/mag_fit.h`: pure floating-point maths, **no ESP-IDF
  headers**, so it compiles and runs on the PC (`tools/host_test/mag_fit_test.c`).
- Solves the general quadric `a x² + b y² + c z² + 2d xy + 2e xz + 2f yz + 2g x +
  2h y + 2i z = 1` by 9x9 normal equations on samples first centred and scaled to
  O(1) (µT-scale coordinates otherwise lose precision).
- Converts the quadric to `(x - c)ᵀ Q (x - c) = 1`, then takes the **symmetric
  positive-definite square root** `M = V·diag(√λ)·Vᵀ` of `Q`. The ellipsoid alone
  determines `M` only up to a left rotation; the physical assumption (soft iron is a
  symmetric PD matrix) picks this branch. Taking a Cholesky factor instead was the
  first version and it quietly mixed an arbitrary rotation into `M`: the centre and
  radius were exact, yet heading error over ±60 deg of tilt was 3.9 deg mean / 8.2 deg
  max. The symmetric root gives 0.000 deg on the same data (regression case 2).
- Outliers are removed with a two-stage LTS scheme (hard trim to the 75 % smallest
  residuals, three iterations, then re-admit points within median + 3·MAD) because
  touching a steel object for a few frames is enough to drag the centre by ~20 uT,
  and a plain "drop beyond 3σ" loop cannot recover from that (residual σ is already
  inflated by the outliers).
- Sanity gates before saving: residual RMS ≤ 20 %, soft-matrix column ratio ≤ 4,
  radius within 5..500 uT. If the fit is rejected the code falls back to min-max and
  logs a warning, so a bad fit never silently overwrites a usable calibration.
- Coverage is still checked independently by the old min/max span ratio (`quality`,
  gate 0.4). A fit on a 25 deg cone "succeeds" with a 0.96 % residual while the centre
  is 19.4 uT off — residual cannot detect missing coverage, the span gate can.
- NVS: blob magic is now `MAG2` (`offset` + 3x3 `soft` + `radius` + `quality` +
  `residual` + `method`). An existing `MAG1` blob is read and converted to
  `soft = diag(scale)`, so upgrading does not force a recalibration, but the boot log
  says so and recommends `RECAL_MAG`.

### Host-side regression test

```
gcc -O2 -Wall -Wextra -I main tools/host_test/mag_fit_test.c main/mag_fit.c -lm \
    -o tools/host_test/mag_fit_test.exe
./tools/host_test/mag_fit_test.exe
```

Synthetic data, deterministic RNG, all six cases pass:

| Case | Result |
|---|---|
| ideal sphere, hard iron only | centre error < 0.05 uT, radius error < 0.2 uT, heading error 0.000 deg |
| soft iron (1.10/0.94/1.03 + 0.06/0.02/0.03 cross-axis) | centre exact, heading error 0.000 deg — **min-max on the same data: 87.5 deg mean / 179.9 deg max** |
| 0.5 uT RMS noise | centre error 0.04 uT, fit residual 0.99 %, heading error 0.12 deg mean / 0.26 deg max |
| 3 % spikes (+80 uT) | centre error 0.04 uT, 485/500 samples kept |
| 25 deg cone only | centre off 19.4 uT (as expected — this is what the span/quality gate is for) |
| 40 vs 39 samples | 40 fits, 39 rejected |

The min-max number is a stress case (6 % cross-axis coupling); the point is not that
real hardware is that bad, but that the old model has a systematic gap that can flip
heading when the horizontal component gets small.

## Four-criteria measurement

Everything below runs on the PC; the only manual part is moving the board.

### Procedure

1. Clean location: no laptop, speaker, PSU or metal within ~1 m. Board level and
   still for the gyro bias window.
   The board's USB-Serial-JTAG enumerates as **COM3** on this machine (it moves if you
   use another USB port); check with
   `[System.IO.Ports.SerialPort]::GetPortNames()` or the device manager.
2. Recalibrate:
   ```
   python tools/att_report.py record --port COM3 --seconds 120 --out logs/cal.log
   ```
   Type `RECAL_MAG` and press Enter. The board reboots, gyro-calibrates, then asks for
   the 15 s rotation; rotate slowly through all orientations (all three axes must
   swing, `画 8 字`). Look for
   `校准完成（椭球拟合）：喂入 N 个样本，存 M，拟合用 U（剔除 K） offset=[...] 半径=... 残差=...% 覆盖度=...`
   and `Mag calibration OK: samples=... method=椭球拟合 quality=... radius=...`.
3. Four criteria in one recording (typing `MARK <label>` while it records is optional,
   the analyzer does not need it):
   ```
   python tools/att_report.py record --port COM3 --seconds 200 --out logs/criteria.log
   ```
   then: board level and hands off → tilt ~30 deg (prop one edge on a book so the hand
   is away) → back to level → rotate 180 deg **without changing the board's position**
   (turn it on a sheet of paper) → hands off.
4. Judge:
   ```
   python tools/att_report.py analyze logs/*.log
   ```
   Exit code 0 = all criteria pass.

### How the tool judges (it changed twice for a reason)

- Segmenting by `MARK` alone was not enough: marks cannot be timed well enough, and any
  recording that contains a reboot breaks the `ATT` millisecond column (it restarts at
  0). The analyzer now finds steady plateaus from the data (±0.6 s least-squares slope),
  ignores the 30 s after a reboot, and stitches reboots with a time offset.
- Plateaus whose \|B\| or dip differs from the others are flagged (`!`) as a different
  magnetic environment and excluded — otherwise a magnetic disturbance reads as a
  heading change.
- `ATT` columns 19..21 are the gyro (deg/s). `Δyaw陀螺` is the Z-gyro integral between
  two plateaus, i.e. how much the board *actually* turned. Judging
  `|Δyaw_mag − Δyaw_gyro|` separates "the operator twisted the board" from "the
  magnetic side is wrong"; the raw `Δyaw` cannot. Without those columns the tool falls
  back to the old absolute checks.
- `dip` (magnetic inclination computed from the accelerometer-tilted field) is an
  invariant of the location and must not change with tilt.

Checks implemented (thresholds are CLI options):

| # | Criterion | Default limit |
|---|---|---|
| 1 | static yaw σ (detrended) | ≤ 0.5 deg |
| 2 | yaw drift | ≤ 2.0 deg/min |
| 3 | \|yaw − magnetic heading\| | ≤ 1.0 deg |
| 4 | tilt 20..45 deg: \|Δyaw_mag − Δyaw_gyro\| | ≤ 2.0 deg |
| 5 | turn ≥ 120 deg: \|Δyaw_mag − Δyaw_gyro\| | ≤ 5.0 deg |
| 6 | mag heading vs gyro integral, any pair | ≤ 2.0 deg |

### Results (2026-10-25, ellipsoid calibration, desk ~0.5-1 m from a laptop)

```
I (17080) mag: 校准完成（椭球拟合）：喂入 358 个样本，存 358，拟合用 332（剔除 26）
              offset=[-28.7, 18.8, 83.8] µT 半径=49.9 µT 残差=0.94% 覆盖度=0.87
```

| Item | Criterion | Measured | Verdict |
|---|---|---|---|
| Magnetic calibration | residual / coverage / radius | 0.94 % / 0.87 / 49.9 µT | pass |
| Static σ, 132 s (`cal.log`) | ≤ 0.5 deg | **0.092 deg** | pass |
| Static σ, 46 s (criteria run) | ≤ 0.5 deg | **0.123 deg** | pass |
| Drift | ≤ 2 deg/min | **−0.11 / +0.22 deg/min** | pass |
| \|yaw − mag heading\| | ≤ 1 deg | **0.07 / 0.23 deg** | pass |
| Tilt 31 deg (gyro-corrected) | ≤ 2 deg | **3.6 deg** | **fail** |
| 180 deg turn | ≤ 5 deg | not measured — the post-turn plateau sat in a different field (\|B\| 47.7 vs 49.1 µT, dip −57.9° vs −52.5°) and was excluded | n/a |

### What the numbers say

- The ellipsoid calibration fixed the old weakness: \|B\| now stays at the stored radius
  (49.9 µT) instead of wandering between 45 and 57 µT.
- Static behaviour is excellent and repeatable: σ 0.09-0.12°, drift ~0.2 deg/min,
  yaw vs magnetic heading 0.07-0.23°.
- Tilt: going into the 30° tilt the heading error is **0.0°** — the measured Δyaw was
  +10.8° and the gyro integral was +10.8°, i.e. the operator genuinely twisted the board
  while tilting. Coming back out of the tilt the heading was **3.6°** off with **0.0°**
  of gyro rotation, so 3.6° is a real tilt-induced heading error, above the 2° limit.
- Most likely cause, in order:
  1. **The accelerometer is not calibrated.** The level reading is
     `ax=+14 mg, ay=−14 mg, az=1010 mg` → ~0.8° of roll/pitch bias and a 1 % scale
     error. A tilt error θ rotates the compensated heading by roughly θ·tan(dip); with
     dip ≈ 54°, 1.6° of tilt error already gives ~2.2°.
  2. Soft-iron residual: dip shifts 2° between level (−54.0°) and 30° tilt (−52.0°);
     the fit residual is 0.94 %, so a small residual is expected.
  3. A small mag-vs-accel mounting skew (1-2°) that the 24-entry axis map cannot
     represent.
- The turn must be redone in a stable field. Rotating the board by hand moved it into
  a different field (dip changed 5.4°), which is a measurement problem, not a firmware
  one.

### Next steps

1. Redo only the turn, keeping the board in the same spot (rotate it on a sheet of
   paper); 1-2 minutes.
2. Then attack the 3.6° tilt error. The single most promising change is a proper
   accelerometer calibration (zero-g offset + scale, 6-face), because the level reading
   already shows ~0.8° of bias. Second step would be estimating a small accel-vs-mag
   mounting rotation.

### First boot after the calibration change (2026-10-25)

The new firmware was built, flashed to the board (COM3, USB-Serial-JTAG) and booted:

```
E (152) icm42688p: I2C 地址 0x68 无应答 ...            <- expected, AP_AD0 high
I (229) icm42688p: ICM42688P 就绪：addr=0x69 WHO_AM_I=0x47 PWR=0x0F GYRO=0x47 ACCEL=0x67
I (1671) icm42688p: 陀螺零偏校准完成 (200 样本)：bias = [-0.444, -0.135, 0.021] °/s
I (1828) mag: BMM350 在线：地址=0x14 chip_id=0x33
W (1877) mag: 已载入旧版 min-max 校准：offset=[-60.7, -3.9, 83.8] µT 半径=45.8 µT
I (1878) mag: 轴向映射：已从 NVS 载入 idx=19
CMD_READY,RECAL_MAG
```

- the `MAG1` -> `MAG2` migration works on real NVS data: the old min-max numbers are
  loaded, `method=min-max`, and the warning asks for `RECAL_MAG`;
- `MARK smoke` -> `MARK,32089,smoke`, and `PING` -> `serial command received: PING`;
- `CMD_READY,RECAL_MAG` is unchanged, so the `pc_viewer` recalibration button still
  enables (checked in `pc_viewer/index.html`, which compares the line exactly);
- one 23 s static capture analysed with `tools/att_report.py`: yaw σ 0.055 deg, drift
  1.46 deg/min, \|yaw − mag heading\| 0.11 deg, all PASS — but \|B\| = 49.6 µT against
  the stored radius 45.8 µT, i.e. the old min-max calibration was indeed the weak link.

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

Superseded by the ellipsoid-fit recalibration and the measurements in *Four-criteria
measurement* above: the stored radius is now fitted (49.9 uT) instead of min-max
(45.8 uT), the tilt check measures 3.6 deg, and only the 180 deg turn still needs a run
in a stable field.
