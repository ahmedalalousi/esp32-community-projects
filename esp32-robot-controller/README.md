# Waveshare ESP32-S3 Robot Controller

A ROS2-based robot control system using a Waveshare ESP32-S3-ETH microcontroller with WiFi/Ethernet connectivity, IMU, ultrasonic sensor, and camera.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Control Station                                │
│  (Mac with Docker / Raspberry Pi)                                           │
│                                                                             │
│  ┌──────────────────────┐    ┌─────────────────────┐    ┌─────────────────┐ │
│  │ teleop_twist_keyboard│    │  waveshare_driver   │    │ web_video_server│ │
│  │                      │    │                     │    │   (optional)    │ │
│  │  Publishes:          │    │  Subscribes:        │    │                 │ │
│  │   /cmd_vel           │───▶│   /cmd_vel          │    │  Serves:        │ │
│  │                      │    │                     │    │   /camera/      │ │
│  └──────────────────────┘    │  Publishes:         │    │   image_raw     │ │
│                              │   /range            │    │   via HTTP      │ │
│                              │   /imu              │    │                 │ │
│                              │   /camera/image_raw │    └─────────────────┘ │
│                              └──────────┬──────────┘                        │
│                                         │                                   │
└─────────────────────────────────────────┼───────────────────────────────────┘
                                          │
                     ┌────────────────────┼────────────────────┐
                     │ TCP :8080          │ TCP :8081          │ HTTP :8082
                     │ Motor commands     │ Sensor data        │ MJPEG stream
                     │ (Pi → ESP32)       │ (ESP32 → Pi)       │ (ESP32 serves)
                     ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Waveshare ESP32-S3-ETH                              │
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ TCP Server  │  │ TCP Client  │  │ HTTP Server │  │    Motor Driver     │ │
│  │ Port 8080   │  │ → Pi:8081   │  │ Port 8082   │  │                     │ │
│  │             │  │             │  │             │  │  4x PWM + 4x DIR    │ │
│  │ Receives:   │  │ Sends:      │  │ Serves:     │  │  PID heading        │ │
│  │  motor cmd  │  │  IMU data   │  │  MJPEG      │  │  correction         │ │
│  │  stop       │  │  range data │  │  stream     │  │                     │ │
│  │  estop      │  │             │  │             │  │                     │ │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│                                                                             │
│  ┌─────────────────────────────────┐  ┌───────────────────────────────────┐ │
│  │         I2C Sensors             │  │           Network                 │ │
│  │                                 │  │                                   │ │
│  │  MPU-9250 (0x68)                │  │  WiFi (primary)                   │ │
│  │   - Accelerometer ±2g           │  │  W5500 Ethernet (failover)        │ │
│  │   - Gyroscope ±250°/s           │  │                                   │ │
│  │   - Magnetometer (AK8963)       │  │                                   │ │
│  │                                 │  │                                   │ │
│  │  URM09 Ultrasonic (0x11)        │  │                                   │ │
│  │   - Range: 2-500cm              │  │                                   │ │
│  └─────────────────────────────────┘  └───────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Repository Structure

```
.
├── esp32-robot-controller/          # ESP-IDF project for ESP32-S3
│   ├── main/
│   │   ├── main.c                   # Application entry point
│   │   ├── network.c/h              # WiFi + Ethernet dual-interface
│   │   ├── motor_driver.c/h         # PWM motor control with PID
│   │   ├── sensors.c/h              # MPU-9250 + URM09 I2C drivers
│   │   ├── tcp_server.c/h           # Command reception (port 8080)
│   │   ├── tcp_client.c/h           # Sensor streaming (port 8081)
│   │   ├── camera_server.c/h        # MJPEG HTTP server (port 8082)
│   │   ├── Kconfig.projbuild        # Menuconfig options
│   │   └── CMakeLists.txt
│   ├── sdkconfig.defaults           # Default configuration
│   └── README.md
│
├── ros2_humble_docker/              # Docker development environment
│   ├── docker-compose.yml
│   ├── Dockerfile
│   ├── entrypoint.sh
│   ├── workspace/
│   │   └── toyRobot/
│   │       └── waveshare_driver/    # ROS2 Python package
│   │           ├── waveshare_driver/
│   │           │   └── driver_node.py
│   │           ├── package.xml
│   │           └── setup.py
│   ├── home/                        # Persistent home directory
│   └── installed/                   # Tracked apt packages
│       └── packages.txt
│
└── docs/                            # Documentation
    ├── SETUP_RASPBERRY_PI.md
    └── HARDWARE_CONNECTIONS.md
```

## Communication Protocol

### Motor Commands (Pi → ESP32, TCP port 8080)

JSON messages, newline-terminated:

```json
{"cmd": "motor", "linear": 0.5, "angular": 0.1}
{"cmd": "stop"}
{"cmd": "estop"}
{"cmd": "mode", "mode": "four_wheel"}
{"cmd": "pid", "kp": 1.5, "ki": 0.2, "kd": 0.05}
{"cmd": "set_motor", "id": 0, "speed": 50}
```

| Command     | Parameters                             | Description          |
|-------------|----------------------------------------|----------------------|
| `motor`     | `linear` (m/s), `angular` (rad/s)      | Set velocity         |
| `stop`      | Graceful stop with PID reset           |                      |
| `estop`     | Emergency stop, immediate PWM kill     |                      |
| `mode`      | `mode`: two_wheel/four_wheel/tracked   | Change drive mode    |
| `pid`       | `kp`, `ki`, `kd` | Update PID gains    |                      |
| `set_motor` | `id` (0-3), `speed` (-100 to 100)      | Direct motor control |
|-----------------------------------------------------------------------------|

### Sensor Data (ESP32 → Pi, TCP port 8081)

JSON messages, newline-terminated:

```json
{"type": "range", "distance": 0.45}
{"type": "imu", "ax": 0.1, "ay": -0.05, "az": 9.81, "gx": 0.0, "gy": 0.0, "gz": 0.01, "mx": 25.0, "my": -10.0, "mz": 40.0, "temp": 25.5}
```

| Type    | Fields           | Units              |
|---------|------------------|--------------------|
| `range` | `distance`       | metres             |
| `imu`   | `ax`, `ay`, `az` | m/s²               |
| `gx`, `gy`, `gz`           | rad/s              |
| `mx`, `my`, `mz`           | µT (MPU-9250 only) |
| `temp`  |                  | °C                 |
|-------------------------------------------------|

### Camera Stream (ESP32 serves, HTTP port 8082)

- **URL:** `http://<esp32_ip>:8082/stream`
- **Format:** MJPEG over HTTP (multipart/x-mixed-replace)
- **Resolution:** Configurable via Kconfig (default: VGA 640×480)

## Hardware Configuration

### Waveshare ESP32-S3-ETH Pinout

| Function        | GPIO   | Notes            |
|-----------------|--------|------------------|
| **W5500 (SPI)** |        |                  |
| SCLK            | 13     |                  |
| MOSI            | 11     |                  |
| MISO            | 12     |                  |
| CS              | 14     |                  |
| INT             | 10     |                  |
| RST             | 9      |                  |
| **I2C Bus**     |        |                  |
| SDA             | 38     | MPU-9250 + URM09 |
| SCL             | 39     |                  |
| **Motor PWM**   |        |                  |
| Front-left PWM  | 4      |                  |
| Front-left DIR  | 5      |                  |
| Front-right PWM | 6      |                  |
| Front-right DIR | 7      |                  |
| Rear-left PWM   | 15     |                  |
| Rear-left DIR   | 16     |                  |
| Rear-right PWM  | 17     |                  |
| Rear-right DIR  | 18     |                  |
| **Reserved**    |        |                  |
| USB             | 19, 20 | Do not use       |
| UART            | 43, 44 | Serial monitor   |
| Flash/PSRAM     | 26-37  | Internal         |
|                 |        |                  |

### I2C Device Addresses

| Device                | Address | Notes                       |
|-----------------------|---------|-----------------------------|
| MPU-9250 (accel/gyro) | 0x68    | AD0 low                     |
| AK8963 (magnetometer) | 0x0C    | Inside MPU-9250, via bypass |
| URM09 (ultrasonic)    | 0x11    | Default, configurable       |
|---------------------------------------------------------------|

## Quick Start

### 1. Flash the ESP32

```bash
cd esp32-robot-controller
idf.py menuconfig   # Configure WiFi SSID/password and Pi IP
idf.py build flash monitor
```

Key menuconfig settings:
- **WiFi Configuration** → Set SSID and password
- **Robot Controller Configuration → TCP Communication → Pi IP address** → Set to your Mac/Pi IP

### 2. Start the Docker Environment (Development on Mac)

```bash
cd ros2_humble_docker
docker-compose up -d
docker exec -it ros2_humble bash
```

### 3. Run the ROS2 Driver

Inside the container:

```bash
cd /ros2_ws
colcon build --packages-select waveshare_driver
source install/setup.bash
ros2 run waveshare_driver driver_node --ros-args \
  -p esp32_ip:=<ESP32_IP> \
  -p camera_url:='http://<ESP32_IP>:8082/stream'
```

### 4. Control the Robot

In a second terminal:

```bash
docker exec -it ros2_humble bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys:
- `i` — forward
- `,` — backward  
- `j` — turn left
- `l` — turn right
- `k` — stop

### 5. Monitor Sensors

```bash
ros2 topic echo /range
ros2 topic echo /imu
ros2 topic hz /camera/image_raw
```

## ROS2 Topics

| Topic               | Type                | Description                            |
|---------------------|---------------------|----------------------------------------|
| `/cmd_vel`          | geometry_msgs/Twist | Velocity commands (subscribed)         |
| `/range`            | sensor_msgs/Range   | Ultrasonic distance (published)        |
| `/imu`              | sensor_msgs/Imu     | IMU data (published)                   |
| `/camera/image_raw` | sensor_msgs/Image   | Camera frames (published)              |
|------------------------------------------------------------------------------------|

## Configuration

### ESP32 Kconfig Options

Access via `idf.py menuconfig`:

| Menu          | Option        | Default      | Description                   |
|---------------|---------------|--------------|-------------------------------|
| WiFi          | SSID          | myssid       | Network name                  |
| WiFi          | Password      | mypassword   | Network password              |
| Motor Driver  | Drive mode    | Four-wheel   | TWO_WHEEL, FOUR_WHEEL, TRACKED|
| Motor Driver  | Max speed     | 3000 mm/s    | Speed at 100% PWM             |
| Motor Driver  | PWM frequency | 20 kHz       | Above audible range           |
| PID           | Enabled       | Yes          | Heading correction            |
| PID           | Kp            | 1.50         | Proportional gain             |
| PID           | Ki            | 0.20         | Integral gain                 |
| PID           | Kd            | 0.05         | Derivative gain               |
| IMU           | Accel range   | ±2g          | ±2g, ±4g, ±8g, ±16g           |
| IMU           | Gyro range    | ±250°/s      | ±250, ±500, ±1000, ±2000      |
| TCP           | Command port  | 8080         | Motor commands                |
| TCP           | Sensor port   | 8081         | Sensor streaming              |
| TCP           | Pi IP         | 192.168.1.100| ROS2 node address             |
| Camera        | Enabled       | No           | Enable camera support         |
|------------------------------------------------------------------------------|

### ROS2 Node Parameters

```bash
ros2 run waveshare_driver driver_node --ros-args \
  -p esp32_ip:=192.168.1.50 \
  -p esp32_command_port:=8080 \
  -p sensor_listen_port:=8081 \
  -p camera_url:='http://192.168.1.50:8082/stream' \
  -p camera_enabled:=true \
  -p camera_fps:=10.0
```

## Troubleshooting

### ESP32 boot loop
- Check serial monitor for error messages
- Common cause: duplicate `esp_event_loop_create_default()` calls
- Ensure network.c and main.c don't both create the event loop

### "Connection refused" from ROS2 node
- Verify ESP32 is connected to network (check serial monitor for IP)
- Ensure ESP32 and Pi/Mac are on the same subnet
- Check firewall settings on Mac/Pi

### No sensor data received
- Verify ESP32 is configured with correct Pi IP address
- Check port 8081 is not blocked
- Test with: `nc -l 8081` on Pi, watch for incoming JSON

### Camera stream lag
- Current implementation has 2-3 second latency
- This is a known limitation of MJPEG over HTTP via OpenCV
- Phase 2 will address this with direct frame transfer

## Version History

| Version | Date     | Changes                                         |
|---------|----------|-------------------------------------------------|
| 0.1.0   | Feb 2026 | Initial release: motor control, sensors, camera |
|----------------------------------------------------------------------|
## Author

Ahmed Al-Alousi

## License

MIT
