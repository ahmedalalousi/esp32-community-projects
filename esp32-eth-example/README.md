# ESP32-S3 Dual Interface Network Example

![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.x-blue)
![Hardware](https://img.shields.io/badge/Hardware-ESP32--S3-green)
![Ethernet](https://img.shields.io/badge/Ethernet-W5500-orange)
![License](https://img.shields.io/badge/License-CC0-lightgrey)

**Working example demonstrating WiFi + W5500 Ethernet working together seamlessly on ESP32-S3.**

This is a minimal, production-ready example showing how to configure and use both WiFi and Ethernet interfaces simultaneously on an ESP32-S3 with automatic failover.

## Author

**Ahmed Al-Alousi**  
December 2025

## Hardware Requirements

- **ESP32-S3 Development Board** (tested with 16MB Flash, 8MB PSRAM)
- **W5500 Ethernet Module** (SPI-based)
- **Jumper wires** for SPI connections

## Tested Hardware

- **ESP32-S3:** 
  - Waveshare ESP32-S3-ETH (16MB Flash, 8MB PSRAM) ✅ Recommended
  - Various ESP32-S3 DevKit boards with USB-C
- **Ethernet Module:** W5500-based modules (commonly available from Waveshare, DFRobot, etc.)

**Note:** This example is specifically configured for ESP32-S3 with **16MB Flash and 8MB PSRAM** (the most common configuration). If your board has different specs, you may need to adjust flash/PSRAM settings.

## Features

✅ **Dual Interface Support:** WiFi and Ethernet work simultaneously  
✅ **Automatic Failover:** Prefers Ethernet when available, falls back to WiFi  
✅ **Non-blocking Connection:** Both interfaces connect asynchronously  
✅ **Clean API:** Simple functions to check status and get IP addresses  
✅ **Production Ready:** Proper error handling and logging  
✅ **Fully Documented:** Every configuration option explained  

## Pin Configuration (W5500 to ESP32-S3)

The following GPIO pins are used for W5500 SPI communication:

| W5500 Pin | ESP32-S3 GPIO | Function | Notes |
|-----------|---------------|----------|-------|
| MOSI      | GPIO 11       | SPI MOSI | Master Out Slave In |
| MISO      | GPIO 12       | SPI MISO | Master In Slave Out |
| SCLK      | GPIO 13       | SPI Clock | 20 MHz clock speed |
| CS        | GPIO 14       | Chip Select | Active low |
| INT       | GPIO 10       | Interrupt | Optional but recommended |
| RST       | GPIO 9        | Reset | Hardware reset line |
| VCC       | 3.3V          | Power | **Do not use 5V** |
| GND       | GND           | Ground | Common ground essential |

### Important Notes

⚠️ **Power Supply:** W5500 requires stable 3.3V supply. Some modules have onboard regulators for 5V, check your module's specifications.

⚠️ **SPI Bus:** The example uses `SPI2_HOST` (HSPI). ESP32-S3 has multiple SPI buses available.

⚠️ **GPIO Selection:** These GPIO pins were chosen to avoid conflicts with:
- USB (GPIO 19-20)
- UART (GPIO 43-44)
- JTAG (GPIO 39-42)
- Flash/PSRAM (GPIO 26-37)

## Quick Start

### 1. Hardware Setup

Connect your W5500 module to ESP32-S3 using the pin configuration above.

**Wiring Checklist:**
```
[ ] MOSI → GPIO 11
[ ] MISO → GPIO 12
[ ] SCLK → GPIO 13
[ ] CS   → GPIO 14
[ ] INT  → GPIO 10
[ ] RST  → GPIO 9
[ ] VCC  → 3.3V (verify module voltage!)
[ ] GND  → GND
[ ] Ethernet cable connected to W5500
[ ] Ethernet cable connected to router/switch
```

### 2. Software Prerequisites

Install ESP-IDF v5.0 or later:

```bash
# Clone ESP-IDF (if not already installed)
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.5.1  # Or latest stable version

# Install ESP-IDF
./install.sh esp32s3

# Source the environment (do this in every terminal)
. ./export.sh
```

### 3. Clone and Configure

```bash
# Clone this example
git clone https://github.com/ahmedalalousi/esp32-community-projects.git
cd esp32s3-dual-interface-example

# Set target (should be automatic from sdkconfig.defaults, but verify)
idf.py set-target esp32s3

# Configure WiFi credentials
idf.py menuconfig
```

**Important:** The project is pre-configured for **ESP32-S3 with 16MB Flash and 8MB PSRAM** (like Waveshare ESP32-S3-ETH). The `sdkconfig.defaults` file automatically sets:
- Target: ESP32-S3
- Flash size: 16MB
- PSRAM: 8MB Octal PSRAM enabled
- USB Serial JTAG console

If you have a different ESP32-S3 variant, you may need to adjust flash/PSRAM settings in menuconfig.

In menuconfig:
1. Navigate to: **WiFi Configuration**
2. Set **WiFi SSID** (your network name)
3. Set **WiFi Password**
4. Save and exit (press `S`, then `Q`)

**The Ethernet configuration is already set in `sdkconfig.defaults`** - you don't need to configure it unless you want to change pins.

### 4. Build and Flash

```bash
# Build the project
idf.py build

# Flash to ESP32-S3
idf.py -p /dev/ttyUSB0 flash

# Monitor output (CTRL-] to exit)
idf.py -p /dev/ttyUSB0 monitor
```

Replace `/dev/ttyUSB0` with your actual serial port:
- **Linux:** `/dev/ttyUSB0` or `/dev/ttyACM0`
- **macOS:** `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`
- **Windows:** `COM3`, `COM4`, etc.

### 5. Expected Output

After flashing, you should see:

```
========================================
 ESP32-S3 Dual Interface Network Example
 WiFi + W5500 Ethernet
========================================
I (xxx) NETWORK: Initialising dual-interface network...
I (xxx) NETWORK: ✓ WiFi initialised (SSID: YourSSID)
I (xxx) NETWORK: ✓ Ethernet (W5500) initialised
I (xxx) NETWORK: ✓ Dual-interface network initialised
I (xxx) MAIN: ✓ System initialised
I (xxx) NETWORK: WiFi connected to AP
I (xxx) NETWORK: ✓ WiFi got IP: 192.168.1.100
I (xxx) NETWORK: Ethernet link up
I (xxx) NETWORK: ✓ Ethernet got IP: 192.168.1.101
I (xxx) MAIN: ========================================
I (xxx) MAIN: Network Status:
I (xxx) MAIN:   WiFi:     Connected (192.168.1.100)
I (xxx) MAIN:   Ethernet: Connected (192.168.1.101)
I (xxx) MAIN:   Active:   Ethernet
I (xxx) MAIN: ========================================
```

## Detailed Menuconfig Setup (If Needed)

If you need to change Ethernet pin configuration or other settings:

### Step 1: Enter Menuconfig

```bash
idf.py menuconfig
```

### Step 2: Configure Ethernet (Component Config → Ethernet)

Navigate: **Component config** → **Ethernet**

#### 2.1 Enable Ethernet Support

1. Select **Support Ethernet**
2. Press `Y` to enable

#### 2.2 Select SPI Ethernet

1. Go to **Ethernet Type**
2. Select **SPI Ethernet**
3. Press `Enter`

#### 2.3 Enable W5500

1. In **SPI Ethernet**, enable **Use W5500 (MAC/PHY)**
2. Press `Y`

#### 2.4 Configure SPI Host

1. Select **Ethernet SPI Host**
2. Choose **SPI2_HOST (HSPI)** (default)

#### 2.5 Configure GPIO Pins

Set the following GPIO numbers:

```
SPI SCLK GPIO: 13
SPI MOSI GPIO: 11
SPI MISO GPIO: 12
SPI CS GPIO:   14
SPI INT GPIO:  10
SPI RST GPIO:  9
SPI Clock Speed: 20 MHz
```

#### 2.6 Configure PHY

1. **PHY Address:** 1 (default for W5500)
2. **PHY Reset GPIO:** 9

### Step 3: Configure WiFi Credentials

Navigate: **WiFi Configuration** (top-level menu)

1. Set **WiFi SSID:** Your network name
2. Set **WiFi Password:** Your WiFi password

### Step 4: Save and Exit

Press `S` to save, then `Q` to quit.

## Troubleshooting

### Build Fails or Won't Boot (CRITICAL - READ FIRST)

**Symptom:** Build errors about flash size, or device boots then crashes/reboots

**Common errors:**
```
E (xxx) spi_flash: Detected size(4MB) doesn't match the size in the binary image header(16MB)
E (xxx) esp_image: Image length X doesn't fit in partition length Y
E (xxx) cpu_start: Failed to verify app image: invalid magic byte
Brownout detector triggered
```

**SOLUTION - Set correct target and flash size:**

```bash
# CRITICAL: Set target FIRST
idf.py set-target esp32s3

# This will regenerate sdkconfig with correct defaults
# Then build
idf.py build
```

**If that doesn't work:**
1. Delete old sdkconfig: `rm sdkconfig`
2. Set target: `idf.py set-target esp32s3`
3. Rebuild: `idf.py build`

**Explanation:**  
The `sdkconfig.defaults` file contains all the correct settings (16MB flash, 8MB PSRAM), but if you built for a different target before (like ESP32), the old `sdkconfig` file will override the defaults. Running `set-target esp32s3` forces regeneration with correct settings.

**Verify your board specs:**
- Waveshare ESP32-S3-ETH: 16MB Flash + 8MB PSRAM ✓ (default config works)
- Other ESP32-S3 boards: Check datasheet and adjust `sdkconfig.defaults` if needed

### Ethernet Not Connecting

**Symptom:** WiFi connects but Ethernet shows "link down"

**Solutions:**
1. **Check wiring** - Use a multimeter to verify connections
2. **Check power** - Ensure W5500 has stable 3.3V (measure with multimeter)
3. **Check Ethernet cable** - Try a different cable
4. **Check router** - Ensure router port is active (LED should light up)
5. **Check GPIO conflicts** - Make sure no other peripherals use these GPIOs
6. **Try different SPI speed** - Reduce `CONFIG_ETH_SPI_CLOCK_MHZ` to 10 or 5

Enable debug logging:
```bash
idf.py menuconfig
# Component config → Ethernet → Set log level to DEBUG
```

### WiFi Not Connecting

**Symptom:** WiFi shows "disconnected, reconnecting..."

**Solutions:**
1. **Verify credentials** - Double-check SSID and password (case-sensitive!)
2. **Check WiFi band** - ESP32-S3 only supports 2.4GHz, not 5GHz
3. **Check router settings** - Ensure WPA2-PSK is enabled
4. **Signal strength** - Move ESP32 closer to router
5. **Router capacity** - Some routers limit number of connections

### Neither Interface Connects

**Solutions:**
1. **Check NVS** - Erase flash: `idf.py erase-flash`
2. **Check power supply** - Use powered USB hub or external 5V supply
3. **Check ESP32-S3** - Try a known-working example first
4. **Rebuild** - Clean build: `idf.py fullclean && idf.py build`

### Wrong Flash/PSRAM Configuration

**Symptom:** Build succeeds but device crashes on boot, or shows errors about flash/PSRAM

**This is CRITICAL for Waveshare ESP32-S3-ETH and similar boards!**

**Solution:**
```bash
# Verify target is set correctly
idf.py set-target esp32s3

# Check current flash size
grep FLASHSIZE sdkconfig

# Should show:
# CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
# CONFIG_ESPTOOLPY_FLASHSIZE="16MB"

# If wrong, run menuconfig:
idf.py menuconfig
# Navigate to: Serial flasher config → Flash size → 16 MB
# Navigate to: Component config → ESP PSRAM → Enable (if not already)
# Save and rebuild
```

**For different ESP32-S3 variants:**
- **4MB Flash, 2MB PSRAM** - Change flash to 4MB, PSRAM to Quad mode
- **8MB Flash, 8MB PSRAM** - Change flash to 8MB
- Check your board's specifications!

### Build Errors

**Error:** `Ethernet not enabled`  
**Solution:** Make sure `sdkconfig.defaults` is present and run `idf.py reconfigure`

**Error:** `GPIO conflict`  
**Solution:** Check if other components (UART, JTAG, etc.) are using the same GPIOs

### Runtime Errors

**Error:** `E (xxx) spi_master: spi_bus_initialize: SPI2 already in use`  
**Solution:** Another component already initialised SPI2. Change to SPI1_HOST. SPI0_HOST is resurved for flash - don't use that.

**Error:** `E (xxx) w5500.mac: w5500_verify_id(210): wrong chip ID`  
**Solution:** 
- Check SPI wiring (especially MISO/MOSI - easy to swap!)
- Reduce SPI clock speed
- Check power supply

## API Reference

### Initialisation

```c
esp_err_t network_init(void);
```
Initialises both WiFi and Ethernet interfaces. Must be called before `network_start()`.

### Starting Connections

```c
esp_err_t network_start(void);
```
Starts connection attempts for both interfaces (non-blocking).

### Checking Connection Status

```c
bool network_is_connected(network_interface_t interface);
```
Returns `true` if specified interface is connected.

**Parameters:**
- `NETWORK_IF_WIFI` - Check WiFi status
- `NETWORK_IF_ETHERNET` - Check Ethernet status

### Getting IP Address

```c
const char* network_get_ip(network_interface_t interface);
```
Returns IP address string, or `NULL` if not connected.

### Getting Active Interface

```c
network_interface_t network_get_active_interface(void);
```
Returns currently active interface (prefers Ethernet over WiFi).

## Project Structure

```
esp32s3-dual-interface-example/
├── CMakeLists.txt              # Top-level CMake file
├── sdkconfig.defaults          # Default configuration (W5500 pins)
├── README.md                   # This file
└── main/
    ├── CMakeLists.txt          # Component CMake file
    ├── Kconfig.projbuild       # WiFi credentials config
    ├── main.c                  # Application entry point
    ├── network.c               # Network implementation
    └── network.h               # Network API header
```

## Configuration Files Explained

### sdkconfig.defaults

Contains **all Ethernet GPIO pin configurations** and essential settings. This file is loaded automatically and eliminates the need for manual menuconfig setup for Ethernet.

**You only need menuconfig for WiFi credentials.**

### Kconfig.projbuild

Provides menuconfig options for WiFi SSID and password.

## Customization

### Changing Interface Priority

Edit `network.c` → `network_get_active_interface()`:

```c
// Prefer WiFi over Ethernet
if (s_wifi_connected) {
    return NETWORK_IF_WIFI;
} else if (s_eth_connected) {
    return NETWORK_IF_ETHERNET;
}
```

### Changing GPIO Pins

Edit `sdkconfig.defaults` and change the GPIO numbers, or use menuconfig:

```
CONFIG_ETH_SPI_SCLK_GPIO=<your_pin>
CONFIG_ETH_SPI_MOSI_GPIO=<your_pin>
# ... etc
```

### Using Different SPI Bus

Change `CONFIG_ETH_SPI_HOST=3` for SPI3_HOST (VSPI).

### Reducing SPI Speed

If you experience reliability issues, reduce clock speed:

```
CONFIG_ETH_SPI_CLOCK_MHZ=10
```

Or try: 5, 8, 12, 15 MHz

## Known Issues

### ESP-IDF Version Compatibility

- **Tested with:** ESP-IDF v5.0, v5.1, v5.2, v5.3, v5.5.1
- **Known issue:** Some v4.x versions have W5500 driver bugs

### W5500 Module Variations

Different W5500 modules may have:
- Different voltage requirements (3.3V vs 5V with onboard regulator)
- Different pin labels (check your module's datasheet!)
- Some modules have the PHY address strapped to 0 instead of 1

If Ethernet doesn't work, try changing:
```
CONFIG_ETH_PHY_ADDR=0
```

## Contributing

Found a bug? Want to improve the example? 

Please report issues or submit pull requests!

## License

This example code is in the Public Domain (or CC0 licensed, at your option).

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.

## Acknowledgements

Thanks to:
- Espressif for ESP-IDF
- The ESP32 community for W5500 driver development
- Everyone who tested and provided feedback

---

For questions or support, please open an issue on the repository.
