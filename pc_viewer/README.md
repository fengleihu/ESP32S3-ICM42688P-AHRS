# Browser attitude viewer

This standalone viewer connects directly to the ESP32-S3 USB serial port using the
Web Serial API. It has no backend, external JavaScript dependencies, or cloud
connection. The page displays a 3D board pose, Roll/Pitch/Yaw instruments, a magnetic
heading compass, sensor/calibration status, and a rolling 60-second angle plot.

## Requirements

- Current Google Chrome or Microsoft Edge on desktop.
- ESP32-S3 flashed with this project's AHRS firmware.
- A local HTTP origin (`localhost`) or HTTPS. Opening `index.html` directly as a
  `file://` URL does not provide Web Serial access.
- Close ESP-IDF Monitor or any other application currently using the board's COM port.

## Run

From the project root in PowerShell:

```powershell
python -m http.server 8000 --directory pc_viewer
```

Open <http://localhost:8000> in Chrome or Edge, select the firmware's serial baud
rate (default `115200`), click **连接串口**, and choose the ESP32-S3 USB serial port
in the browser's device picker. The page reads `ATT,` CSV lines; non-ATT ESP-IDF log
lines are ignored. Click **断开** before closing the page.

The browser only receives serial data while the page is open. It does not save or
upload telemetry.

## Request magnetometer recalibration

While connected, wait for the firmware's `CMD_READY,RECAL_MAG` marker to enable
**请求磁力计重校准**, then click and confirm the prompt. The viewer sends `RECAL_MAG`
followed by a newline; the firmware records a one-time request and restarts.
Reconnect the serial port after reboot. Keep the board still during gyro
bias calibration, then slowly rotate the complete assembly through many orientations
for the prompted calibration interval (15 seconds by default). Do this away from
phones, laptops, speakers, magnets, and large metal objects. The command preserves
the stored axis map and unrelated NVS values.

## Measure yaw settling time

After rotating the board, stop it and immediately click **标记停止并开始计时**.
The viewer records the first ATT frame after the click, then frames at or just after
1, 3, and 5 seconds. The table shows actual browser elapsed time, firmware uptime
milliseconds, fused angles, and direct magnetic heading. Click **复制记录** to copy
a CSV snippet for analysis.

The firmware's first ATT field is already uptime in milliseconds; the browser timer
adds a user-marked stop event so the settling interval can be measured directly.

## Compare the model with geographic north

The 3D view shows separate geographic-north and magnetic-north arrows. Enter the
local magnetic declination in the **磁偏角 东+** field: east declination is positive
and west declination is negative. Find the current value for the device's location
using a geomagnetic-field calculator such as the
[NOAA magnetic declination calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination).
The value is saved in this browser. The default `0°` means no declination has been
entered, so the two north arrows overlap and should not be treated as a geographic
reference unless the local declination is actually zero.

## Data format

The parser consumes the firmware's `ATT` format. The first five values after `ATT`
are uptime milliseconds and Roll/Pitch/Yaw in degrees. Additional values provide
magnetometer presence/calibration flags, accelerometer samples, and magnetic-heading
diagnostics. The viewer requires at least the first 10 values and gracefully leaves
optional diagnostics unused if absent.

The 3D board model uses the project body frame: `+X` forward (photo top edge), `+Y`
left, and `+Z` out of the component side. Displayed yaw remains relative to magnetic
north; the declination setting adjusts only the geographic-north reference arrow.
