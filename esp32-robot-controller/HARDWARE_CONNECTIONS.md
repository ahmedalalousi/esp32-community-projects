# Hardware Connections Guide

Wiring diagrams and connection details for the Waveshare ESP32-S3-ETH robot controller.

## Components

| Component | Model | Interface | Notes |
|-----------|-------|-----------|-------|
| Microcontroller | Waveshare ESP32-S3-ETH | — | 16MB Flash, 8MB PSRAM, integrated W5500 |
| IMU | MPU-9250 | I2C | 9-axis: accel + gyro + magnetometer |
| Ultrasonic | DFRobot URM09 (SEN0304) | I2C | 2-500cm range |
| Motors | 4× DC gear motors | PWM + DIR | Voltage depends on your motors |
| Motor Driver | L298N or similar | — | Dual H-bridge |
| Camera | OV2640 | DVP | Optional |

---

## I2C Bus Connections

Both sensors share the same I2C bus.

```
ESP32-S3-ETH          MPU-9250           URM09
┌─────────┐         ┌─────────┐       ┌─────────┐
│         │         │         │       │         │
│  GPIO38 ├────┬────┤ SDA     │   ┌───┤ SDA     │
│  (SDA)  │    │    │         │   │   │         │
│         │    └────┼─────────┼───┘   │         │
│         │         │         │       │         │
│  GPIO39 ├────┬────┤ SCL     │   ┌───┤ SCL     │
│  (SCL)  │    │    │         │   │   │         │
│         │    └────┼─────────┼───┘   │         │
│         │         │         │       │         │
│  3.3V   ├────┬────┤ VCC     │   ┌───┤ VCC     │
│         │    │    │         │   │   │         │
│         │    └────┼─────────┼───┘   │         │
│         │         │         │       │         │
│  GND    ├────┬────┤ GND     │   ┌───┤ GND     │
│         │    │    │         │   │   │         │
│         │    └────┼─────────┼───┘   │         │
└─────────┘         └─────────┘       └─────────┘
```

### I2C Pull-up Resistors

The ESP32-S3 has internal pull-ups enabled in the firmware, but for reliable operation at 400kHz, external pull-ups are recommended:

- **SDA:** 4.7kΩ to 3.3V
- **SCL:** 4.7kΩ to 3.3V

### I2C Addresses

| Device | Default Address | Configurable |
|--------|-----------------|--------------|
| MPU-9250 | 0x68 | 0x69 (AD0 high) |
| AK8963 (inside MPU-9250) | 0x0C | No |
| URM09 | 0x11 | Yes, via register |

---

## Motor Driver Connections

### Using L298N Dual H-Bridge

The L298N can drive 2 motors per board. For 4-wheel drive, use two L298N boards or a 4-channel driver.

```
ESP32-S3-ETH          L298N (Board 1)           Motors
┌─────────┐         ┌───────────────┐         ┌─────────┐
│         │         │               │         │         │
│  GPIO4  ├─────────┤ ENA (PWM)     │         │ Front   │
│         │         │               ├─────────┤ Left    │
│  GPIO5  ├─────────┤ IN1 (DIR)     │         │         │
│         │         │ IN2 (¬DIR)    ├───┐     └─────────┘
│         │         │               │   │
│  GPIO6  ├─────────┤ ENB (PWM)     │   │     ┌─────────┐
│         │         │               ├───┼─────┤ Front   │
│  GPIO7  ├─────────┤ IN3 (DIR)     │   │     │ Right   │
│         │         │ IN4 (¬DIR)    ├───┘     │         │
│         │         │               │         └─────────┘
│         │         │               │
│  GND    ├─────────┤ GND           │
│         │         │               │
└─────────┘         └───────────────┘


ESP32-S3-ETH          L298N (Board 2)           Motors
┌─────────┐         ┌───────────────┐         ┌─────────┐
│         │         │               │         │         │
│  GPIO15 ├─────────┤ ENA (PWM)     │         │ Rear    │
│         │         │               ├─────────┤ Left    │
│  GPIO16 ├─────────┤ IN1 (DIR)     │         │         │
│         │         │ IN2 (¬DIR)    ├───┐     └─────────┘
│         │         │               │   │
│  GPIO17 ├─────────┤ ENB (PWM)     │   │     ┌─────────┐
│         │         │               ├───┼─────┤ Rear    │
│  GPIO18 ├─────────┤ IN3 (DIR)     │   │     │ Right   │
│         │         │ IN4 (¬DIR)    ├───┘     │         │
│         │         │               │         └─────────┘
│         │         │               │
│  GND    ├─────────┤ GND           │
│         │         │               │
└─────────┘         └───────────────┘
```

### Direction Control Logic

The firmware uses a single DIR pin per motor (not IN1/IN2 pairs). Wire as follows:

| DIR Pin State | Motor Direction |
|---------------|-----------------|
| HIGH | Forward |
| LOW | Reverse |

For L298N, connect:
- ESP32 DIR pin → IN1
- IN2 connected to inverted DIR (via NOT gate) OR tie IN2 low and only use forward/brake

**Simpler approach:** Use a motor driver with PWM + DIR inputs (e.g., TB6612, DRV8833).

### Motor Pin Summary

| Motor | PWM GPIO | DIR GPIO | LEDC Channel |
|-------|----------|----------|--------------|
| Front-left | 4 | 5 | 0 |
| Front-right | 6 | 7 | 1 |
| Rear-left | 15 | 16 | 2 |
| Rear-right | 17 | 18 | 3 |

---

## Power Supply

### Voltage Requirements

| Component | Voltage | Current (typical) |
|-----------|---------|-------------------|
| ESP32-S3-ETH | 5V via USB or 3.3V direct | 200-500mA |
| MPU-9250 | 3.3V | 3.5mA |
| URM09 | 3.3-5V | 20mA |
| Motors | 6-12V (depends on motor) | 0.5-2A each |
| L298N | 5V logic, 7-35V motor | — |

### Recommended Setup

```
Battery (11.1V LiPo)
       │
       ├──────────────────┬────────────────────┐
       │                  │                    │
       ▼                  ▼                    ▼
   ┌───────┐         ┌─────────┐         ┌─────────┐
   │ 5V    │         │ L298N   │         │ L298N   │
   │ BEC   │         │ Motor   │         │ Motor   │
   │       │         │ Supply  │         │ Supply  │
   └───┬───┘         └─────────┘         └─────────┘
       │
       ▼
   ESP32-S3-ETH (5V input)
       │
       ├── 3.3V ──► MPU-9250
       │
       └── 3.3V ──► URM09
```

**Important:**
- Use a BEC (Battery Elimination Circuit) or buck converter to provide clean 5V to ESP32
- Do NOT power ESP32 from L298N's 5V output — it's often noisy
- Connect all grounds together (common ground)

---

## Camera Connections (Optional)

The Waveshare ESP32-S3-ETH has a camera connector. If using an OV2640:

| ESP32 GPIO | OV2640 Pin |
|------------|------------|
| 40 | XCLK |
| 21 | SIOD (I2C SDA) |
| 47 | SIOC (I2C SCL) |
| 48 | D7 |
| 46 | D6 |
| 45 | D5 |
| 42 | D4 |
| 41 | D3 |
| 2 | D2 |
| 1 | D1 |
| 0 | D0 |
| 3 | VSYNC |
| 8 | HREF |
| 22 | PCLK |

**Note:** Camera uses the ESP32's second I2C controller (SCCB), separate from the sensor I2C bus.

---

## GPIO Conflict Avoidance

### Reserved GPIOs (Do NOT Use)

| GPIO | Reason |
|------|--------|
| 0 | Boot mode (if using camera, OK after boot) |
| 9-14 | W5500 Ethernet SPI |
| 19-20 | USB |
| 26-37 | Flash/PSRAM (internal) |
| 43-44 | UART0 (serial monitor) |

### Safe GPIOs for Motors

The default motor pins (4-7, 15-18) are safe. If you need alternatives:

| Available GPIOs | Notes |
|-----------------|-------|
| 1, 2, 3, 8 | May conflict with camera |
| 21, 22, 40-42, 45-48 | May conflict with camera |

---

## Connector Recommendations

For a robust robot, use:

| Connection  | Connector Type                                      |
|-------------|-----------------------------------------------------|
| I2C sensors | JST-PH 4-pin or Qwiic/STEMMA QT                     |
| Motors      | XT60 or Anderson Powerpole (power), JST-XH (signal) |
| Battery     | XT60                                                |
| ESP32 power | Barrel jack or USB-C                                |
|-------------|-----------------------------------------------------|

---

## Testing Procedure

### 1. I2C Bus Scan

Flash a simple I2C scanner or check boot logs:

```
I (xxx) SENSORS: I2C master initialised (SDA=38, SCL=39, 400kHz)
I (xxx) SENSORS: MPU-9250 detected (WHO_AM_I=0x71)
I (xxx) SENSORS: AK8963 magnetometer initialised (100Hz, 16-bit)
I (xxx) SENSORS: URM09 initialised at 0x11 (range=500cm, passive mode)
```

### 2. Motor Test

Use the `set_motor` command to test each motor individually:

```json
{"cmd": "set_motor", "id": 0, "speed": 50}
```

Motors should spin:
- id 0: Front-left
- id 1: Front-right
- id 2: Rear-left
- id 3: Rear-right

### 3. Sensor Data

Monitor the TCP sensor stream:

```bash
nc <esp32_ip> 8081
```

Or check ROS2 topics:

```bash
ros2 topic echo /range
ros2 topic echo /imu
```

---

## Schematic (Simplified)

```
                                    ┌──────────────────────────────────────┐
                                    │        Waveshare ESP32-S3-ETH        │
                                    │                                      │
     ┌──────────┐                   │  ┌────────┐        ┌────────┐        │
     │ MPU-9250 │◄──────I2C────────►│  │ GPIO38 │ SDA    │ GPIO4  │ FL_PWM ├───►
     └──────────┘                   │  │ GPIO39 │ SCL    │ GPIO5  │ FL_DIR ├───►
                                    │  └────────┘        │ GPIO6  │ FR_PWM ├───►
     ┌──────────┐                   │                    │ GPIO7  │ FR_DIR ├───►  To Motor
     │  URM09   │◄──────I2C────────►│                    │ GPIO15 │ RL_PWM ├───►  Drivers
     └──────────┘                   │                    │ GPIO16 │ RL_DIR ├───►
                                    │                    │ GPIO17 │ RR_PWM ├───►
     ┌──────────┐                   │                    │ GPIO18 │ RR_DIR ├───►
     │ OV2640   │◄─────DVP─────────►│  (camera pins)     └────────┘        │
     └──────────┘                   │                                      │
                                    │  ┌────────────────────────────────┐  │
         ┌──────────────────────────┼──┤ W5500 Ethernet (integrated)    │  │
         │                          │  │ SPI: GPIO 9-14                 │  │
         ▼                          │  └────────────────────────────────┘  │
    ┌─────────┐                     │                                      │
    │ Network │                     │  ┌────────────────────────────────┐  │
    │ Switch  │                     │  │ USB-C (power + serial)         │  │
    └─────────┘                     │  └────────────────────────────────┘  │
                                    └──────────────────────────────────────┘
```
