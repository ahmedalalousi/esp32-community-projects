# ESP32 Community Projects

Development projects for the **Waveshare ESP32-S3-ETH** board, built on the ESP-IDF framework.

## Projects

### esp32-eth-example

Dual-interface networking reference implementation. Provides simultaneous WiFi and W5500 Ethernet connectivity with automatic failover. All SPI and network parameters are configurable via `idf.py menuconfig`.

This project serves as the network foundation reused by other projects in the repository.

### esp32-robot-controller

ROS2-integrated robot firmware for a four-wheel skid-steer platform. Communicates with a Raspberry Pi running ROS2 Humble via TCP, receiving motor velocity commands and streaming sensor telemetry and camera footage back to the ROS2 computation graph.

**Key features:**

- Skid-steer differential drive with configurable two-wheel, four-wheel, and tracked modes
- PID heading correction using IMU yaw rate feedback
- I2C sensor subsystem: MPU-6050 IMU and DFRobot URM09 ultrasonic distance sensor
- TCP command server (port 8080) for motor velocity commands from ROS2
- TCP sensor client (port 8081) streaming IMU and range data to ROS2
- Optional MJPEG camera server (port 8082) for video streaming
- Full Kconfig-driven configuration — no hardcoded pin assignments or parameters

The network stack is inherited from `esp32-eth-example`. The corresponding ROS2 node (`waveshare_driver`) runs in a Docker-hosted ROS2 Humble environment on the Pi.

## Hardware

All projects target the **Waveshare ESP32-S3-ETH** with:

- ESP32-S3 dual-core, 16 MB flash, 8 MB PSRAM (octal SPI)
- Integrated W5500 Ethernet controller via SPI2
- WiFi 802.11 b/g/n

## Build

Each project follows the standard ESP-IDF workflow:

```bash
cd <project-directory>
idf.py set-target esp32s3
idf.py menuconfig    # Configure WiFi credentials, GPIO pins, etc.
idf.py build
idf.py flash monitor
```

## GPIO Allocation (esp32-robot-controller)

| Subsystem        | GPIOs               | Notes                          |
|-----------------|---------------------|--------------------------------|
| W5500 Ethernet  | 9, 10, 11, 12, 13, 14 | SPI2 bus + interrupt + reset |
| Motor driver    | 4, 5, 6, 7, 15, 16, 17, 18 | PWM + direction (4 motors) |
| I2C sensors     | 38, 39              | Shared bus (IMU + ultrasonic)  |
| USB             | 19, 20              | Reserved                       |
| UART            | 43, 44              | Reserved                       |
| Flash/PSRAM     | 26-32               | Octal SPI (unavailable)        |
| Camera          | Various             | Disabled by default            |

All GPIO assignments are configurable via `idf.py menuconfig`.

## Author

Dr Ahmed Al-Alousi — Cuneiform Ltd
