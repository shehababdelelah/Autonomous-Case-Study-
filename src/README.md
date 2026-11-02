# CAS Box

The box ESP32 hosts a Wi-Fi access point and phone web page, reads the box sensors, and controls
the WAVE ROVER over Wi-Fi. The rover firmware is separate and must not be changed.

## Build and upload with PlatformIO

The project is configured in the root `platformio.ini` for the
`esp32doit-devkit-v1` board with the Arduino framework. Open the project root (the folder
containing `platformio.ini`) in VS Code with the PlatformIO IDE extension. PlatformIO installs
the ESP32 platform and libraries declared in that file during the first build.

1. Set `BOX_ID` in `src/main.cpp` to the kit number (1–10).
2. Connect the box by USB and select its serial port in PlatformIO if it is not detected.
3. Build: `pio run -e esp32doit-devkit-v1`
4. Upload: `pio run -e esp32doit-devkit-v1 -t upload`
5. Open the serial monitor at 115200 baud: `pio device monitor -b 115200`

In VS Code, the equivalent actions are **PlatformIO: Build**, **PlatformIO: Upload**, and
**PlatformIO: Monitor**. If several serial devices are connected, specify the port, for example
`pio run -e esp32doit-devkit-v1 -t upload --upload-port COM5`.

The configuration uses `huge_app.csv`, the ESP32 Arduino framework and the libraries in
`platformio.ini`; do not install those libraries manually through Arduino IDE. A CH340 USB
driver may be needed on Windows if the board does not appear as a COM port.

## FreeRTOS task layout

`setup()` initializes shared hardware and starts the application tasks. Arduino's `loop()` now
only yields; it does not perform application work.

## Source file layout

Each module's `.hpp` declares its class and state; its `.cpp` implements the module's functions.
`main.cpp` creates the modules, coordinates sensor polling, and combines their readings for the
web page.

| Files | Responsibility |
|---|---|
| `LidarSensor.cpp/.hpp` | LD20 parsing/filtering, the LiDAR scene snapshot, and LiDAR CSV logging. |
| `IMUSensor.cpp/.hpp` | BNO055 startup, heading/orientation readings, and calibration diagnostics. |
| `Ultrasonics.cpp/.hpp` | Five HC-SR04 sensors and their background measurement task. |
| `RoverLink.cpp/.hpp` | Wi-Fi communication with the rover. |
| `GPSsesnor.cpp/.hpp` | NEO-6M GPS UART and decoded readings. |
| `LCDDisplay.cpp/.hpp` | SSD1306 OLED output. |
| `WEBpage.cpp/.hpp` | Wi-Fi access point, HTTP routes, web page, and drive-mode inputs. |
| `main.cpp` | Module instances, FreeRTOS application tasks, and cross-module coordination. |

| Task | Core | Period / work |
|---|---:|---|
| `sensor` | 1 | Polls LiDAR/GPS/IMU, publishes sensor JSON, and updates the OLED. The OLED and IMU share I2C, so this task owns both. |
| `ui` | 1 | Updates web status values and UI timing statistics. |
| `drive` | 1 | Owns rover setpoints; selects LiDAR wall-follow, timed movement, or manual input every 20 ms. |
| `ultrasonic` | 0 | Measures the five HC-SR04 sensors in sequence. |
| `roverLink` | 0 | Finds the rover and sends drive/status HTTP requests; its own 500 ms setpoint deadman remains active. |
| Async web server | managed by ESPAsyncWebServer | Serves the page and handles `/drive` and `/values` asynchronously. |

The sensor task calls each sensor module directly and combines their readings for the web page
every 250 ms. Web drive requests are accepted by `WEBpage`; the page refreshes an active timed
drive command every 150 ms, and the drive task forwards the accepted setpoint to `RoverLink`.
Manual commands retain the 400 ms timeout, and RoverLink's deadman remains active.

## LiDAR left-wall following

The main page includes explicit Start/Stop controls and live diagnostics. It follows the left wall
using bins 63–72 (approximately 252–292 degrees) and checks the front using bins 86–89 and 0–3.
This assumes the current web-view convention of bin 0 at the rover front; verify the physical
front/left mapping before driving. The mode uses the median of at least four left-sector bins and
the nearest of at least two front-sector returns. It stops on stale/unavailable LiDAR, insufficient
returns, a missing/too-far/too-close wall, or a front return at or below 30 cm.

Control values in `main.cpp` are target wall distance 130 cm, maximum accepted wall distance
245 cm, stop distance 30 cm, wall deadband 5 cm, correction 0.0025 per cm, and correction cap
0.08. The page's wall-follow speed defaults to 0.15 and can be set from 0.10 to the existing
`MAX_SPEED` of 0.5. For example, a 164 cm wall return produces a capped correction of -0.08;
at speed 0.15 the requested left/right wheel speeds are 0.07/0.23. A return inside the 5 cm
deadband produces no steering correction.

Only one movement mode may drive at a time: wall-follow, timed movement recording, or manual
drive. The page sends a wall-follow heartbeat every 250 ms; if that heartbeat is absent for more
than 750 ms, the mode stops. Wall-follow measurements and stop reasons appear under its controls
and in `/values`. A broad displayed LiDAR segment is not the wall-follow measurement: the
controller uses only the specified left bins, so verify their readings on the page while
stationary before starting.

Test first with the rover secured so its wheels cannot carry it away, then on a clear level area
with a straight wall about 130 cm to its left. Start at the default low speed, watch the wall/front
distances, correction, wheel commands, and stop reason, and use the webpage Stop control. Confirm
that a front obstacle closer than 30 cm stops the rover. Do not use the wall-follow mode on a ramp
until its scan sectors and stop behavior have been checked under supervision.

## Right-turn test

The main page's **Test right turn (3 s)** button starts a separate supervised test: the left wheel
is commanded at `MAX_SPEED` and the right wheel at `-MAX_SPEED` for up to 3 seconds. The
`driveTask()` owns the motor commands and ends the test automatically; **Stop turn test** or the
drive pad's Stop button cancels it early. The test is exclusive with wall-follow, timed movement
recording, and manual pad commands. It does not use LiDAR to select or confirm the turn angle, so
the rover may rotate by more or less than 90 degrees depending on surface, battery, and traction.
Test only with the rover supervised in a clear area, starting with its wheels safely raised or
the rover otherwise secured.

The normal directional drive-pad buttons use a separate **5–60 second duration slider** (default
5 seconds). Tap a direction once to move for that duration; releasing does not stop the rover.
The page sends periodic refreshes while the timer is active so the 400 ms web deadman remains
effective. The Stop button cancels immediately, and losing page visibility or focus also stops.
The rover receives the requested wheel commands directly; no IMU heading correction is applied.

The application tasks are pinned to core 1 so they do not compete directly with the existing
ultrasonic and rover-link tasks on core 0. Keep blocking sensor/network operations out of the
drive task: rover communication already runs in `roverLink`.

## IMU readings and calibration

The IMU module uses the BNO055's own Euler and calibration readings. There is no custom north
alignment or angle conversion. The sensor task calls `IMUSensor::update()` about every 100 ms.
That function reads the angles and calibration, saves one sample, and prints it to the Serial
Monitor. Connect at **115200 baud** to see messages like:

`[IMU] t=12345 ms | heading/yaw=90.0 deg | roll=1.0 deg | pitch=-2.0 deg | cal sys/gyro/accel/mag=3/3/3/3`

In simple terms, the module's functions are:

| Function | What it does |
|---|---|
| `IMUSensor::begin()` | Starts I2C on GPIO 21/22 and tries to start the BNO055 at address `0x28`. |
| `IMUSensor::_startIfPresent()` | Selects NDOF mode; retries initialization every 5 seconds if the sensor is missing. |
| `IMUSensor::_readEuler()` | Reads BNO055 Euler x/y/z as heading/yaw, roll, and pitch. |
| `IMUSensor::_readCalibration()` | Reads system, gyro, accelerometer, and magnetometer calibration levels. |
| `IMUSensor::update()` | Calls the two read functions, saves one complete sample, and prints it to Serial. |
| `IMUSensor::read()` | Returns one safe copy of the latest readings for the webpage, logger, and drive task. |

Euler values are degrees, directly as reported by the BNO055: heading/yaw, roll, and pitch.
Calibration order is system / gyro / accelerometer / magnetometer; each value ranges from 0
(uncalibrated) to 3 (fully calibrated). The firmware displays these live values; it does not save
or restore calibration offsets.

The main page shows the current three angles, sample time, and four calibration levels. IMU values
are for display and logging only; they do not steer the rover or stop a movement run.

## LiDAR labeled-reading API

The `/test` web page has been removed. The main page at `/` continues to display the existing
LiDAR and IMU diagnostics. The LiDAR labeled-reading API and CSV logger remain available:

- `GET /test/lidar/live` returns the Front, Right, and Left sector-median ranges and logger
  health. Each range is the median of five adjacent 4-degree snapshot bins; at least two valid
  bins are required. Direction names follow the configured LiDAR mounting orientation.
- `POST /test/lidar/save` accepts form fields `direction` (`Front`, `Right`, or `Left`) and
  `type` (`Wall`, `Pillar`, or `Panel`). The asynchronous handler copies the sensor snapshot and
  queues the record for the dedicated FreeRTOS logger task.
- `GET /test/lidar/records` returns the latest 32 saved observations as JSON.
- `GET /test/lidar.csv` downloads the CSV file from LittleFS.

CSV columns are `uptime_ms,direction,object_type,distance_cm`. The timestamp is milliseconds since
ESP32 boot, not calendar time (the box does not currently have a configured real-time clock).
Object labels are operator-supplied; the firmware does not automatically classify objects. Saving
is rejected if the selected sector has no return or the LiDAR packet rate/snapshot is not ready.
The CSV is limited to 16 KiB.

The existing `huge_app.csv` partition table includes a 896 KiB filesystem partition. The logger
mounts LittleFS on that partition and formats that partition only if LittleFS cannot mount it.
Formatting erases files in that filesystem partition. This project did not previously use the
filesystem for application data.

To exercise the API, keep the rover stationary and prevent the motors from moving. Point the
LiDAR at a known object, inspect `/test/lidar/live`, POST a labeled reading to
`/test/lidar/save`, and verify it through `/test/lidar/records` or download `/test/lidar.csv`.
Compare the measurement against a tape-measured distance. This logger does not autonomously
identify objects or control the rover.

## Live LiDAR returns on the main page

The **Surroundings** card plots the current 90 LiDAR bins around a rover marker and lists measured
return segments with side, clockwise angle from the front, median range, and number of 4-degree
bins. It also shows the number of valid bins within the 3 m display range, the nearest return, and
the age of the latest sensor snapshot. Adjacent bins are grouped only when their ranges are
continuous (difference less than 30 cm or 25% of the nearer range); this is a display grouping,
not identification of a wall, pillar, or panel. At most 24 segments are listed at a time.

The display uses the configured scan frame: bin 0 is rendered as rover-front and bins progress
clockwise. Confirm the physical Front/Right/Left alignment on the stationary rover before relying
on the directional labels.

## Movement sensor CSV

The main page at `/` has **Record + drive forward (60 s)**, **Stop movement + recording**,
**Download IMU CSV**, and **Download LiDAR CSV**. Set the drive-speed slider before starting.
After confirmation, the drive task commands the same selected speed to both wheels for up to
60 seconds. IMU readings are not used to correct steering, and IMU availability or calibration
does not affect the motor command. The manual drive-pad commands are unchanged. The sensor task
continues queuing synchronized IMU/LiDAR samples every 250 ms (about 4 rows per second). Stop ends
movement and recording early. Recording also stops if the CSV logger reports a storage or queue
error. Async web handlers do not read sensors or write the CSV files.

Samples go into two files, both stored in LittleFS and downloaded separately:

- `/imu_euler_data.csv` (`/imu.csv`): columns are `uptime_ms`, `run_start_ms`, `imu_ok`,
  `heading_yaw_deg`, `roll_deg`, `pitch_deg`, `sys_cal`, `gyro_cal`, `accel_cal`, and `mag_cal`.
- The previous-format `/imu_data.csv`, if present from an earlier firmware, is preserved and
  can be downloaded from `/imu-previous.csv` until the movement CSV clear action is used.
- `/lidar_data.csv` (`/lidar.csv`): `uptime_ms`, `run_start_ms`, LiDAR readiness and packet rate,
  explicit front/right/left sector medians in cm, plus all 90 bins (`range_000_cm` through
  `range_356_cm`) clockwise in 4-degree steps, with front at bin 0. A distance of `0` means no
  reliable return in that bin.

The shared `run_start_ms` and `uptime_ms` columns let you match samples across the two CSV files.
Each new run appends to the existing files; it does not erase previous runs. Download links are
enabled after recording stops and queued writes finish. The IMU CSV is capped at 128 KiB and the
LiDAR CSV at 700 KiB to leave room in the filesystem partition. A full file or logger error stops
the timed movement and is reported on the page.

**Delete saved movement CSVs** removes `/imu_euler_data.csv`, `/imu_data.csv`, `/lidar_data.csv`, and the old combined
`/movement_data.csv` file after confirmation, then recreates empty IMU and LiDAR CSVs with headers.
The action is rejected while a run, write, or queued snapshot is active. It does not erase the
manually labeled `/lidar_test.csv` or any other LittleFS files.

## Use

1. Switch on the **box first**, then the rover (the rover looks for its box only while booting).
2. Join Wi-Fi **`CAS-Box-NN`**, password **`CASboxNN`** (NN is the two-digit kit number).
3. Open `http://192.168.(10+N).1`; for example, kit 8 uses `http://192.168.18.1`.
4. Set the drive-button duration (5–60 seconds), then tap a direction. The rover stops when the
   duration expires; press Stop to cancel sooner. The stop/deadman logic also stops stale commands.

The Wi-Fi channel is 1 for odd kit IDs and 11 for even kit IDs. IMU angles are the direct BNO055
Euler readings; heading direction and zero depend on BNO055 fusion/calibration state and are not
converted to the previous vehicle-frame convention.

## Safety and troubleshooting

- Keep the kit-specific Wi-Fi naming scheme so teams do not interfere with each other.
- Do not remove the web timeout, RoverLink deadman, or rover heartbeat stop behavior.
- If the page says `Rover: searching`, turn on the box before restarting the rover and check
  `BOX_ID`.
- If no access point appears, check the serial output for `[WEBpage] AP ...`.
- If no COM port appears, use a USB data cable and check the CH340 driver.
- Sensor JSON is printed periodically; the IMU and OLED may take several seconds to initialize.
