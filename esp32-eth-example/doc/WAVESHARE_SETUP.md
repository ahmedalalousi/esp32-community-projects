# Waveshare ESP32-S3-ETH Quick Setup Guide

**This guide is specifically for the Waveshare ESP32-S3-ETH development board.**

Product: https://www.waveshare.com/esp32-s3-eth.htm

---

## Board Specifications

The Waveshare ESP32-S3-ETH has:
- **MCU:** ESP32-S3-WROOM-1-N16R8 module
- **Flash:** 16MB
- **PSRAM:** 8MB Octal PSRAM
- **Ethernet:** W5500 chip (SPI interface)
- **WiFi:** 2.4GHz 802.11 b/g/n
- **USB:** Native USB (no CH340 needed!)

---

## Critical First Step: Set Target

**BEFORE building, you MUST set the target to ESP32-S3:**

```bash
cd esp32s3-dual-interface-example
idf.py set-target esp32s3
```

**Why?** If you skip this step or built for a different ESP32 variant before, the build will use wrong flash/PSRAM settings and will either:
- Fail to build
- Build but fail to boot
- Boot but crash randomly

---

## Waveshare ESP32-S3-ETH Pin Mapping

The example is **pre-configured** for Waveshare's pin layout:

### Ethernet (W5500)
| Function | GPIO | Waveshare Pin |
|----------|------|---------------|
| MOSI     | 11   | IO11          |
| MISO     | 12   | IO12          |
| SCLK     | 13   | IO13          |
| CS       | 14   | IO14          |
| INT      | 10   | IO10          |
| RST      | 9    | IO9           |

**No wiring needed** - W5500 is already connected on the board!

### Power
- **5V USB-C:** Powers the board
- **3.3V Output:** Available on pin header for external devices
- **Ethernet:** Powered from board (no external supply needed)

---

## Quick Start (Waveshare Specific)

### 1. Install ESP-IDF

```bash
# Install ESP-IDF v5.0 or later
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.5.1
./install.sh esp32s3
. ./export.sh
```

### 2. Clone and Set Target

```bash
git clone <your-repo-url>
cd esp32s3-dual-interface-example

# CRITICAL: Set target FIRST
idf.py set-target esp32s3
```

### 3. Configure WiFi

```bash
idf.py menuconfig
```

Navigate to: **WiFi Configuration**
- Set **WiFi SSID**: Your network name
- Set **WiFi Password**: Your WiFi password
- Save (press `S`) and exit (press `Q`)

**That's it!** Ethernet is already configured for Waveshare's pin layout.

### 4. Build and Flash

```bash
# Build
idf.py build

# Find your USB port
ls /dev/tty.usbmodem*   # macOS
ls /dev/ttyACM*         # Linux
# Windows: Check Device Manager for COM port

# Flash (replace with your port)
idf.py -p /dev/ttyACM0 flash

# Monitor
idf.py -p /dev/ttyACM0 monitor
```

**Exit monitor:** Press `CTRL-]`

---

## Expected Output

After flashing, you should see:

```
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
...
I (xxx) NETWORK: Initialising dual-interface network...
I (xxx) NETWORK: ✓ WiFi initialised (SSID: YourSSID)
I (xxx) NETWORK: ✓ Ethernet (W5500) initialised
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

Both WiFi and Ethernet should get IP addresses within 5-10 seconds.

---

## Waveshare Board Features Used

### Native USB
The board uses ESP32-S3's native USB for programming and serial console:
- **No CH340 chip needed**
- Faster upload speeds
- More reliable serial communication
- Configured automatically in `sdkconfig.defaults`

### On-board W5500
The W5500 Ethernet controller is **already soldered** on the board and connected internally:
- No external wiring needed
- No jumpers to configure
- Just plug in Ethernet cable and go!

### LED Indicators
- **Power LED:** Red, always on when powered
- **WiFi LED:** Blue, blinks during WiFi activity
- **Ethernet LED:** Green, on when Ethernet link is up

---

## Waveshare-Specific Troubleshooting

### "Device not found" when flashing

**Symptom:** `serial.serialutil.SerialException: could not open port`

**Solution:**
1. Make sure board is connected via USB-C
2. Press and hold **BOOT** button
3. Press and release **RESET** button
4. Release **BOOT** button
5. Try flashing again

### Ethernet not working but WiFi works

**Check:**
1. **Ethernet cable plugged in** to RJ45 connector
2. **Router port active** (router's LED should light up)
3. **Board's Ethernet LED** should be green when cable is connected
4. Try different Ethernet cable

### Neither WiFi nor Ethernet works

**Most likely:** Wrong target or flash size

**Solution:**
```bash
# Clean everything
idf.py fullclean
rm sdkconfig

# Set target
idf.py set-target esp32s3

# Rebuild
idf.py build

# Flash
idf.py -p /dev/ttyACM0 flash monitor
```

### Boot loop or brownout

**Symptom:** Device keeps resetting, shows brownout errors

**Causes:**
1. **Poor USB power** - Try powered USB hub or different USB port
2. **Wrong PSRAM config** - Make sure you ran `set-target esp32s3`

**Check power:**
```
# Should NOT see this error:
E (xxx) esp_core_dump_flash: Core dump flash config is corrupted! CRC=0x... instead of 0x...
E (xxx) cpu_start: Failed to verify app image: invalid magic byte
Brownout detector was triggered
```

If you see these, it's a power or configuration issue.

---

## Verifying Configuration

After running `idf.py set-target esp32s3`, verify:

```bash
grep CONFIG_IDF_TARGET sdkconfig
# Should show: CONFIG_IDF_TARGET="esp32s3"

grep CONFIG_ESPTOOLPY_FLASHSIZE sdkconfig
# Should show: CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y

grep CONFIG_SPIRAM sdkconfig
# Should show: CONFIG_SPIRAM=y
```

---

## Hardware Variants

Waveshare makes several variants:

| Model | Flash | PSRAM | Notes |
|-------|-------|-------|-------|
| ESP32-S3-ETH | 16MB | 8MB | Standard (this config) |
| ESP32-S3-ETH-4MB | 4MB | 2MB | Need to change flash size! |

If you have the 4MB variant, edit `sdkconfig.defaults`:
```
# Change from:
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="16MB"

# To:
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="4MB"
```

---

## Performance Tips

### 1. Ethernet is Faster
The code automatically prefers Ethernet over WiFi when both are connected:
- **Ethernet:** ~90 Mbps (W5500 limitation)
- **WiFi:** ~20-40 Mbps (typical 2.4GHz)

### 2. Reduce Power Consumption
If running on battery, disable unused interface:
```c
// In network.c, comment out:
// ESP_ERROR_CHECK(esp_wifi_start());  // Disable WiFi
// or
// ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));  // Disable Ethernet
```

### 3. Increase SPI Speed (Advanced)
In `sdkconfig.defaults`, you can try increasing W5500 SPI speed:
```
# Change from:
CONFIG_ETH_SPI_CLOCK_MHZ=20

# To (if stable):
CONFIG_ETH_SPI_CLOCK_MHZ=40
```

**Warning:** Higher speeds may be unstable. Test thoroughly!

---

## Pinout Reference

Full Waveshare ESP32-S3-ETH pinout:

```
          ┌─────────────────────────────┐
          │                             │
      3V3 ○                             ○ GND
      GND ○                             ○ IO1
       EN ○                             ○ IO2
      IO3 ○    ESP32-S3-WROOM-1         ○ IO42
      IO4 ○        N16R8                ○ IO41
      IO5 ○                             ○ IO40
      IO6 ○     W5500 Ethernet          ○ IO39
      IO7 ○      (On-board)             ○ IO38
     IO15 ○                             ○ IO37
     IO16 ○                             ○ IO36
     IO17 ○      [USB-C]                ○ IO35
     IO18 ○                             ○ IO0
      IO8 ○      [RJ45]                 ○ IO45
     IO19 ○                             ○ IO48
     IO20 ○                             ○ IO47
      3V3 ○                             ○ IO21
      GND ○                             ○ IO14 (W5500 CS)
      TX0 ○                             ○ IO13 (W5500 SCLK)
      RX0 ○                             ○ IO12 (W5500 MISO)
          │                             ○ IO11 (W5500 MOSI)
          │                             ○ IO10 (W5500 INT)
          │                             ○ IO9  (W5500 RST)
          │                             ○ IO46
          │                             ○ 5V
          │                             ○ GND
          └─────────────────────────────┘
```

**Note:** GPIO 9-14 are used by W5500 - **do not use for other peripherals!**

---

## Additional Resources

- **Waveshare Wiki:** https://www.waveshare.com/wiki/ESP32-S3-ETH
- **Datasheet:** Available on Waveshare product page
- **ESP32-S3 Datasheet:** https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
- **W5500 Datasheet:** https://www.wiznet.io/product-item/w5500/

---

## Getting Help

If you have issues specific to the Waveshare board:

1. **Check Waveshare's forum:** https://www.waveshare.com/forum
2. **Check ESP32 forum:** https://esp32.com
3. **Open issue on this repository**

When reporting issues, include:
- Board variant (ESP32-S3-ETH vs ESP32-S3-ETH-4MB)
- ESP-IDF version: `idf.py --version`
- Build command used
- Complete error output

---

**Happy networking with Waveshare ESP32-S3-ETH! 🚀**
