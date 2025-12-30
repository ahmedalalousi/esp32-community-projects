# W5500 Ethernet Initialization Guide

**Complete guide to initialising W5500 Ethernet on ESP32-S3**

This document covers critical pain points discovered during development and testing, along with proven solutions.

---

## Table of Contents

1. [Critical Pain Points](#critical-pain-points)
2. [The Solution: Template Code](#the-solution-template-code)
3. [Quick Start Checklist](#quick-start-checklist)
4. [Detailed Troubleshooting](#detailed-troubleshooting)
5. [Hardware Considerations](#hardware-considerations)

---

## Critical Pain Points

### 1. Missing W5500-Specific SPI Configuration ⚠️ **CRITICAL**

**Problem:** W5500 requires specific SPI protocol parameters that generic ESP-IDF examples don't include.

**Symptoms:**
```
E (xxx) w5500.mac: esp_eth_mac_new_w5500(929): invalid argument
MAC creation FAILED - W5500 not responding
```

**Root Cause:** Missing `.command_bits = 16` and `.address_bits = 8` in SPI device configuration.

**Impact:** W5500 will not respond at all - complete failure to initialise.

**Solution:**
```c
spi_device_interface_config_t spi_devcfg = {
    .command_bits = 16,        // REQUIRED for W5500 protocol
    .address_bits = 8,         // REQUIRED for W5500 protocol
    .mode = 0,
    .clock_speed_hz = CONFIG_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
    .queue_size = 20,
    .spics_io_num = CONFIG_ETH_SPI_CS_GPIO,
};
```

**Why This Happens:** The W5500 uses a specific SPI communication protocol that differs from standard SPI devices. Without these bit configurations, the ESP32 cannot communicate with the chip at all.

---

### 2. NULL MAC Configuration Pointer ⚠️ **CRITICAL**

**Problem:** Passing `NULL` as the second parameter to `esp_eth_mac_new_w5500()`.

**Symptoms:** Same as above - "invalid argument" error, MAC creation returns NULL.

**Root Cause:** ESP-IDF's W5500 driver requires a valid `eth_mac_config_t` structure pointer.

**Impact:** Complete initialisation failure, same symptoms as missing SPI configuration.

**Solution:**
```c
// ❌ WRONG:
esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, NULL);

// ✅ CORRECT:
eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
```

**Why This Happens:** Many ESP-IDF examples for other Ethernet PHYs accept NULL for default configuration, but W5500 driver specifically requires this structure.

---

### 3. Kconfig Symbol Names vs ESP-IDF Component Enable

**Problem:** Custom Kconfig symbols (like `CONFIG_ETH_SPI_MOSI_GPIO`) don't automatically enable ESP-IDF's ethernet component.

**Symptoms:**
```
fatal error: esp_eth.h: No such file or directory
#include "esp_eth.h"
```

**Root Cause:** Kconfig.projbuild didn't use `select` statements to enable ESP-IDF's built-in ETH component.

**Impact:** Compilation failure when Ethernet is disabled in menuconfig.

**Solution:**
```kconfig
menu "Ethernet Configuration"
    config ENABLE_ETHERNET
        bool "Enable Ethernet (W5500)"
        default y
        select ETH_ENABLED              # Enable ESP-IDF ETH component
        select ETH_USE_SPI_ETHERNET     # Enable SPI Ethernet
        select ETH_SPI_ETHERNET_W5500   # Enable W5500 driver
        help
            Enable W5500 Ethernet module for wired network connectivity.
```

**Why This Happens:** ESP-IDF's component system requires explicit enabling of the ethernet component. Custom Kconfig options don't automatically trigger this.

---

### 4. Flash Size and Target Mismatch

**Problem:** Building without running `idf.py set-target esp32s3` first causes flash/PSRAM misconfiguration.

**Symptoms:**
```
E (xxx) spi_flash: Detected size(4MB) doesn't match binary(16MB)
E (xxx) esp_image: Image length X doesn't fit in partition length Y
Brownout detector was triggered
```

**Root Cause:** Old `sdkconfig` from a different target (or default ESP32 target) overrides settings in `sdkconfig.defaults`.

**Impact:** Device won't boot, or boots then crashes immediately.

**Solution:**
```bash
# ALWAYS run this FIRST for new projects or when changing targets
idf.py set-target esp32s3

# If issues persist, force clean rebuild:
rm sdkconfig
idf.py set-target esp32s3
idf.py fullclean
idf.py build
```

**Why This Happens:** The `sdkconfig` file (if present) takes precedence over `sdkconfig.defaults`. When building for a different chip, old settings persist.

---

### 5. Conditional Compilation Issues

**Problem:** Ethernet-related includes and code need proper conditional compilation guards.

**Symptoms:** Compilation errors when Ethernet is disabled, or bloated binary when not used.

**Solution:** Wrap all Ethernet code properly:
```c
#ifdef CONFIG_ENABLE_ETHERNET
#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#endif

// Later in code:
#ifdef CONFIG_ENABLE_ETHERNET
    // Ethernet initialisation code here
#else
    ESP_LOGI(TAG, "Ethernet disabled");
#endif
```

**Why This Happens:** The ETH component and its headers are only available when enabled in the build system.

---

### 6. Hardware Issues (Not Code-Related)

**Problem:** Poor connections, inadequate power, signal integrity issues.

**Symptoms:** Intermittent failures, works sometimes but not others, MAC creation fails randomly.

**Common Issues:**
1. **Poor GND connection** - Single most common hardware issue
2. **Unstable 3.3V supply** - W5500 requires stable power
3. **Long wires** - Causes signal integrity issues at 20MHz SPI
4. **MOSI/MISO swapped** - Very easy mistake to make
5. **Wrong voltage** - Some W5500 modules need 5V (with onboard regulator)

**Solution:**
1. Use multimeter to verify solid GND connection
2. Measure 3.3V at W5500 VCC pin under load
3. Use short wires (< 10cm if possible)
4. Double-check MOSI/MISO aren't swapped
5. Check your module's datasheet for voltage requirements
6. Try lower SPI clock speed (10MHz or 5MHz)

---

## The Solution: Template Code

A complete, documented template is provided in `w5500_init_template.c` which includes:

✅ All critical SPI configuration parameters  
✅ Proper MAC configuration  
✅ Complete error handling with actionable messages  
✅ Kconfig.projbuild template with `select` statements  
✅ sdkconfig.defaults template for ESP32-S3  
✅ Inline documentation explaining WHY each setting exists  
✅ Troubleshooting guide  
✅ Usage examples  

**Key principle:** Every "DO NOT REMOVE" comment in the template exists because someone (us!) spent hours debugging when it was missing.

---

## Quick Start Checklist

For new W5500 projects:

- [ ] Run `idf.py set-target esp32s3` **FIRST**
- [ ] Use Kconfig template with `select` statements
- [ ] Include `.command_bits = 16` in SPI config
- [ ] Include `.address_bits = 8` in SPI config  
- [ ] Pass `&mac_config`, not `NULL`, to `esp_eth_mac_new_w5500()`
- [ ] Wrap all ETH code in `#ifdef CONFIG_ENABLE_ETHERNET`
- [ ] Verify wiring (especially MOSI/MISO, easy to swap!)
- [ ] Check GND connection with multimeter
- [ ] Verify stable 3.3V power supply
- [ ] Start with 20MHz SPI clock, reduce to 10MHz if issues

---

## Detailed Troubleshooting

### "invalid argument" Error

**Full error message:**
```
E (xxx) w5500.mac: esp_eth_mac_new_w5500(929): invalid argument
E (xxx) esp_eth: esp_eth_driver_install(209): can't set eth->mac or eth->phy to null
```

**Causes (in order of likelihood):**
1. Missing `.command_bits = 16` and/or `.address_bits = 8`
2. Passing `NULL` for MAC config instead of `&mac_config`
3. Hardware not connected or not powered
4. Wrong GPIO pin numbers
5. Incompatible SPI clock speed

**Debug steps:**
```c
// Add before MAC creation:
ESP_LOGI(TAG, "SPI Config - command_bits: %d, address_bits: %d", 
         spi_devcfg.command_bits, spi_devcfg.address_bits);
ESP_LOGI(TAG, "GPIO - MOSI:%d MISO:%d SCLK:%d CS:%d", 
         CONFIG_ETH_SPI_MOSI_GPIO, CONFIG_ETH_SPI_MISO_GPIO,
         CONFIG_ETH_SPI_SCLK_GPIO, CONFIG_ETH_SPI_CS_GPIO);

esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
if (mac == NULL) {
    ESP_LOGE(TAG, "MAC creation failed - check configuration above");
}
```

---

### "esp_eth.h: No such file or directory"

**Cause:** ESP-IDF's ETH component not enabled.

**Solution:** Add to your Kconfig.projbuild:
```kconfig
config ENABLE_ETHERNET
    bool "Enable Ethernet (W5500)"
    select ETH_ENABLED
    select ETH_USE_SPI_ETHERNET
    select ETH_SPI_ETHERNET_W5500
```

Then run:
```bash
rm sdkconfig
idf.py reconfigure
idf.py build
```

---

### Flash Size Mismatch / Brownout

**Symptoms:**
```
E (xxx) spi_flash: Detected size(4MB) doesn't match the size in the binary image header(16MB)
Brownout detector was triggered
```

**Solution:**
```bash
rm sdkconfig
idf.py set-target esp32s3
idf.py build
```

**Verify configuration:**
```bash
grep CONFIG_IDF_TARGET sdkconfig
# Should show: CONFIG_IDF_TARGET="esp32s3"

grep CONFIG_ESPTOOLPY_FLASHSIZE sdkconfig
# Should show: CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
```

---

### Works on Waveshare But Not Custom Board

**Likely causes:**
1. Different SPI host number (try 1 instead of 2, or vice versa)
2. Different GPIO pins
3. Poor wiring/connections
4. Inadequate power supply

**Try:**
```bash
idf.py menuconfig
# Ethernet Configuration → SPI Host
# Change from 2 to 1 (or vice versa)

# Also try:
# SPI Clock Speed → Change from 20 to 10 MHz
```

---

### Intermittent Failures

**Symptoms:** Sometimes works, sometimes doesn't.

**Causes:**
1. Poor GND connection (most common)
2. Borderline power supply
3. EMI/signal integrity at high SPI speeds
4. Loose wire connections

**Solutions:**
1. Check all connections with multimeter
2. Add 0.1µF ceramic capacitor near W5500 VCC pin
3. Use shorter wires (< 10cm)
4. Reduce SPI clock speed to 10 or 5 MHz
5. Use twisted pair for MOSI/MISO lines

---

## Hardware Considerations

### SPI Host Selection

Different ESP32-S3 boards may use different SPI hosts:

- **Waveshare ESP32-S3-ETH:** SPI2_HOST (value = 2)
- **Other boards:** May need SPI2_HOST (1) or SPI3_HOST (2)

**Note:** SPI1_HOST (0) is reserved for flash and cannot be used.

If W5500 doesn't respond, try different host values in menuconfig.

---

### GPIO Pin Selection

**Safe GPIOs on ESP32-S3:**
- ✅ GPIO 1-18, 21, 38-42, 45-48

**AVOID these GPIOs:**
- ❌ GPIO 0 (boot mode selection)
- ❌ GPIO 3 (JTAG)
- ❌ GPIO 19-20 (USB)
- ❌ GPIO 26-37 (Flash/PSRAM - NEVER use!)
- ❌ GPIO 43-44 (UART0 console)

---

### Power Requirements

**W5500 Power Specifications:**
- Operating voltage: 3.3V
- Typical current: 132mA (active)
- Peak current: 160mA (during transmission)

**Common issues:**
- USB port power may be insufficient (especially with WiFi also active)
- Long USB cables can cause voltage drop
- Some W5500 modules have onboard 5V→3.3V regulators

**Solutions:**
- Use powered USB hub
- Use external 5V power supply
- Measure voltage at W5500 VCC pin (should be 3.3V ± 0.1V)

---

### SPI Clock Speed

**Recommended speeds:**
- **20 MHz:** Good balance, works with most setups
- **10 MHz:** More reliable with longer wires
- **5 MHz:** Maximum reliability, debug/troubleshooting

**Maximum supported:** 40 MHz (but often unstable)

Start with 20 MHz, reduce if experiencing issues.

---

### Common Wiring Mistakes

1. **MOSI and MISO swapped** - Check twice!
2. **Missing GND connection** - Must share common ground
3. **CS not connected** - Required for SPI communication
4. **INT not connected** - Optional but recommended for performance
5. **RST not connected** - Optional but helps with reliable initialisation

---

## Best Practices

### During Development

1. **Always set target first:** `idf.py set-target esp32s3`
2. **Use template code** - Don't try to simplify "redundant" settings
3. **Add debug logging** - Helps identify exactly where failure occurs
4. **Test one interface at a time** - WiFi first, then add Ethernet
5. **Check with multimeter** - Verify all connections before debugging code

### For Production

1. **Add proper error handling** - Don't just use ESP_ERROR_CHECK
2. **Implement reconnection logic** - Handle cable unplugged/replugged
3. **Monitor link status** - Register ETH_EVENT handlers
4. **Add watchdog timers** - Recover from hung states
5. **Test edge cases** - Unplug cable, power cycle, etc.

---

## Additional Resources

- **ESP-IDF Documentation:** https://docs.espressif.com/projects/esp-idf/
- **W5500 Datasheet:** https://www.wiznet.io/product-item/w5500/
- **Waveshare ESP32-S3-ETH Wiki:** https://www.waveshare.com/wiki/ESP32-S3-ETH
- **Template Code:** See `w5500_init_template.c` in this repository

---

## Version History

- **v1.0** (December 2025) - Initial guide based on extensive testing and debugging
  - Tested on Waveshare ESP32-S3-ETH
  - Tested on ESP32-S3-DevKitC-1 with external W5500 module
  - Covers ESP-IDF v5.x

---

## Contributing

Found additional pain points or solutions? Please contribute:
1. Open an issue describing the problem and solution
2. Submit a pull request with updates to this guide
3. Include hardware details and ESP-IDF version

---

**Author:** Ahmed Al-Alousi  
**License:** CC0 (Public Domain)  
**Last Updated:** December 2025
