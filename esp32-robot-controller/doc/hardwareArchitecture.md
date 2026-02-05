┌─────────────────────────────────────────────────────────────────┐
│                    Raspberry Pi (ROS2 Humble)                   │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐ │
│  │ teleop_node  │  │ waveshare    │  │ web_video_server       │ │
│  │ (keyboard)   │  │ _bridge node │  │                        │ │
│  └──────┬───────┘  └──────┬───────┘  └────────────────────────┘ │
│         │                 │                                     │
│         │ /cmd_vel        │ /range, /imu, /camera/image_raw     │
│         ▼                 ▼                                     │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    waveshare_driver node                    ││
│  │     (translates ROS2 ↔ TCP/UDP to Waveshare)                ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
                              │ WiFi (TCP or UDP)
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Waveshare ESP32-S3                           │
│                    (Simple firmware, no ROS2)                   │
│  - Receive: motor commands (JSON or binary)                     │
│  - Send: sensor readings (JSON or binary)                       │
│  - Serve: MJPEG stream on HTTP port                             │
└─────────────────────────────────────────────────────────────────┘
