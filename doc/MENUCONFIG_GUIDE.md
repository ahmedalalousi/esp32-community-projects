# ESP32-S3 W5500 Ethernet - Complete Menuconfig Guide

**Complete step-by-step instructions for configuring W5500 Ethernet on ESP32-S3 via menuconfig.**

This guide shows **exactly** which menu items to navigate, what values to enter, and what each setting does.

---

## Quick Summary

**If you use `sdkconfig.defaults` (included in this project), you DON'T need to do this!**

This guide is for those who want to:
- Understand what each setting does
- Configure a different board
- Change GPIO pins
- Troubleshoot configuration issues

---

## Prerequisites

```bash
# Make sure you're in the project directory
cd esp32s3-dual-interface-example

# Run menuconfig
idf.py menuconfig
```

You'll see a blue and grey text-based menu interface.

**Navigation:**
- **Arrow keys** (↑↓) - Move between options
- **Enter** - Select/enter a menu
- **Spacebar** or **Y** - Enable an option (shows `[*]` or `<*>`)
- **N** - Disable an option (shows `[ ]` or `< >`)
- **Esc** or **Q** - Go back to previous menu
- **S** - Save configuration
- **?** - Show help for current option

---

## Part 1: Essential Ethernet Configuration

### Step 1: Enter Component Configuration

From the main menu:

```
┌─────────────────────────────────────────────────────────────────┐
│ Arrow keys navigate the menu. <Enter> selects submenus ---> (or│
│ empty submenus ----). Highlighted letters are hotkeys. Pressing│
│ <Y> includes, <N> excludes, <M> modularizes features. Press    │
│ <Esc><Esc> to exit, <?> for Help, </> for Search.              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│        Build type  --->                                         │
│        Bootloader config  --->                                  │
│        Security features  --->                                  │
│        Serial flasher config  --->                              │
│        Partition Table  --->                                    │
│    --> Component config  --->                  ◄── SELECT THIS  │
│        Compatibility options  --->                              │
│        ...                                                      │
└─────────────────────────────────────────────────────────────────┘
```

**Action:** Navigate down to **Component config** and press **Enter**

---

### Step 2: Enter Ethernet Menu

```
┌─────────────────────────────────────────────────────────────────┐
│                     Component config                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│        Driver configurations  --->                              │
│        ESP System Settings  --->                                │
│        IDF Logging  --->                                        │
│    --> Ethernet  --->                         ◄── SELECT THIS   │
│        ESP-Driver:ADC  --->                                     │
│        ...                                                      │
└─────────────────────────────────────────────────────────────────┘
```

**Action:** Navigate to **Ethernet** and press **Enter**

---

### Step 3: Enable Ethernet Support

```
┌─────────────────────────────────────────────────────────────────┐
│                        Ethernet                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    [ ] Support Ethernet                      ◄── ENABLE THIS    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Action:** Press **Y** or **Spacebar** to enable

**Result:** The checkbox should show `[*]`

After enabling, more options appear:

```
┌─────────────────────────────────────────────────────────────────┐
│                        Ethernet                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    [*] Support Ethernet                                         │
│          Ethernet Type (SPI Ethernet MAC)  --->  ◄── SELECT THIS│
│    [ ]   Support ESP32 internal EMAC controller                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

### Step 4: Select SPI Ethernet

**Action:** Navigate to **Ethernet Type** and press **Enter**

```
┌─────────────────────────────────────────────────────────────────┐
│                      Ethernet Type                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    ( ) Internal EMAC                                            │
│    (X) SPI Ethernet MAC               ◄── SELECT THIS (X)       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Action:** 
1. Navigate to **SPI Ethernet MAC**
2. Press **Enter** or **Spacebar** to select it
3. The radio button should show `(X)`
4. Press **Esc** to go back

**Result:** You're back in the Ethernet menu with new options

---

### Step 5: Enter SPI Ethernet Configuration

```
┌─────────────────────────────────────────────────────────────────┐
│                        Ethernet                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    [*] Support Ethernet                                         │
│          Ethernet Type (SPI Ethernet MAC)  --->                 │
│          SPI Ethernet Configuration  --->   ◄── SELECT THIS     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Action:** Navigate to **SPI Ethernet Configuration** and press **Enter**

---

### Step 6: Enable W5500 Support

```
┌─────────────────────────────────────────────────────────────────┐
│                SPI Ethernet Configuration                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    [ ] Use DM9051                                               │
│    [ ] Use KSZ8851SNL                                           │
│    [ ] Use W5500 (MAC/PHY)            ◄── ENABLE THIS           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Action:** 
1. Navigate to **Use W5500 (MAC/PHY)**
2. Press **Y** or **Spacebar**

**Result:** `[*] Use W5500 (MAC/PHY)`

After enabling W5500, more configuration options appear below.

---

### Step 7: Configure SPI Host

```
┌─────────────────────────────────────────────────────────────────┐
│                SPI Ethernet Configuration                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    [*] Use W5500 (MAC/PHY)                                      │
│          Ethernet SPI Host (SPI2_HOST (HSPI))  --->             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Action:** Navigate to **Ethernet SPI Host** and press **Enter**

```
┌─────────────────────────────────────────────────────────────────┐
│                    Ethernet SPI Host                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    ( ) SPI1_HOST (SPI)                                          │
│    (X) SPI2_HOST (HSPI)               ◄── SELECT THIS           │
│    ( ) SPI3_HOST (VSPI)                                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Explanation:**
- **SPI1_HOST** - Reserved for flash memory
- **SPI2_HOST (HSPI)** - **Use this** for W5500 (recommended)
- **SPI3_HOST (VSPI)** - Alternative if SPI2 conflicts with other peripherals

**Action:** Select **SPI2_HOST (HSPI)** and press **Esc**

---

### Step 8: Configure GPIO Pins

Back in SPI Ethernet Configuration menu, you'll now see GPIO options:

```
┌─────────────────────────────────────────────────────────────────┐
│                SPI Ethernet Configuration                       │
├─────────────────────────────────────────────────────────────────┤
│    [*] Use W5500 (MAC/PHY)                                      │
│          Ethernet SPI Host (SPI2_HOST (HSPI))  --->             │
│    (13) SPI SCLK GPIO number                 ◄── CONFIGURE      │
│    (11) SPI MOSI GPIO number                 ◄── CONFIGURE      │
│    (12) SPI MISO GPIO number                 ◄── CONFIGURE      │
│    (14) SPI CS GPIO number                   ◄── CONFIGURE      │
│    (10) Interrupt GPIO number                ◄── CONFIGURE      │
│    (20) Amount of SPI read/write operations  (leave default)    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**For each GPIO setting:**

#### SPI SCLK GPIO (Clock)
1. Navigate to **(XX) SPI SCLK GPIO number**
2. Press **Enter**
3. Type: **13**
4. Press **Enter**

#### SPI MOSI GPIO (Master Out Slave In)
1. Navigate to **(XX) SPI MOSI GPIO number**
2. Press **Enter**
3. Type: **11**
4. Press **Enter**

#### SPI MISO GPIO (Master In Slave Out)
1. Navigate to **(XX) SPI MISO GPIO number**
2. Press **Enter**
3. Type: **12**
4. Press **Enter**

#### SPI CS GPIO (Chip Select)
1. Navigate to **(XX) SPI CS GPIO number**
2. Press **Enter**
3. Type: **14**
4. Press **Enter**

#### Interrupt GPIO
1. Navigate to **(XX) Interrupt GPIO number**
2. Press **Enter**
3. Type: **10**
4. Press **Enter**

**Important GPIO Notes:**

⚠️ **Avoid these GPIOs on ESP32-S3:**
- **GPIO 0** - Boot mode selection
- **GPIO 3** - JTAG
- **GPIO 19-20** - USB
- **GPIO 26-37** - Flash/PSRAM (CRITICAL - never use!)
- **GPIO 43-44** - UART0 (serial console)

✅ **Safe GPIOs for peripherals:** 1-18, 21, 38-42, 45-48

**Result after configuration:**
```
┌─────────────────────────────────────────────────────────────────┐
│                SPI Ethernet Configuration                       │
├─────────────────────────────────────────────────────────────────┤
│    [*] Use W5500 (MAC/PHY)                                      │
│          Ethernet SPI Host (SPI2_HOST (HSPI))  --->             │
│    (13) SPI SCLK GPIO number              ✓                     │
│    (11) SPI MOSI GPIO number              ✓                     │
│    (12) SPI MISO GPIO number              ✓                     │
│    (14) SPI CS GPIO number                ✓                     │
│    (10) Interrupt GPIO number             ✓                     │
└─────────────────────────────────────────────────────────────────┘
```

---

### Step 9: Configure SPI Clock Speed

Scroll down in the same menu:

```
┌─────────────────────────────────────────────────────────────────┐
│                SPI Ethernet Configuration                       │
├─────────────────────────────────────────────────────────────────┤
│    ...                                                          │
│    (20) Ethernet SPI clock speed (MHz)   ◄── CONFIGURE THIS    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Action:**
1. Navigate to **Ethernet SPI clock speed (MHz)**
2. Press **Enter**
3. Type: **20**
4. Press **Enter**

**Explanation:**
- **20 MHz** - Recommended for most setups (good balance)
- **10 MHz** - Use if you have connection issues or long wires
- **40 MHz** - Maximum, may be unstable with poor wiring

---

### Step 10: Configure PHY Address and Reset GPIO

Still in SPI Ethernet Configuration, scroll down more:

```
┌─────────────────────────────────────────────────────────────────┐
│                SPI Ethernet Configuration                       │
├─────────────────────────────────────────────────────────────────┤
│    ...                                                          │
│    (1)  PHY Address                      ◄── CHECK THIS         │
│    (9)  PHY Reset GPIO number            ◄── CONFIGURE THIS     │
│    (500) PHY Reset Assertion Time (ms)   (leave default)        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### PHY Address
**Action:**
1. Navigate to **(X) PHY Address**
2. Press **Enter**
3. Type: **1** (default for most W5500 modules)
4. Press **Enter**

**Note:** Some W5500 modules are strapped to address 0. If Ethernet doesn't work, try changing this to 0.

#### PHY Reset GPIO
**Action:**
1. Navigate to **(X) PHY Reset GPIO number**
2. Press **Enter**
3. Type: **9**
4. Press **Enter**

**Explanation:** This GPIO performs hardware reset of the W5500 chip during initialisation.

---

### Step 11: Exit SPI Ethernet Configuration

**Action:** Press **Esc** to go back to the main Ethernet menu

---

## Part 2: WiFi Configuration

### Step 12: Exit to Main Menu

From the Ethernet menu:

**Action:** Press **Esc** repeatedly until you're back at the top-level menu

---

### Step 13: Configure WiFi Credentials

From the main menu:

```
┌─────────────────────────────────────────────────────────────────┐
│                     ESP-IDF Configuration                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│        Build type  --->                                         │
│        Bootloader config  --->                                  │
│        Security features  --->                                  │
│    --> WiFi Configuration  --->           ◄── SELECT THIS       │
│        Component config  --->                                   │
│        ...                                                      │
└─────────────────────────────────────────────────────────────────┘
```

**Action:** Navigate to **WiFi Configuration** and press **Enter**

```
┌─────────────────────────────────────────────────────────────────┐
│                   WiFi Configuration                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    (myssid) WiFi SSID                    ◄── CONFIGURE THIS     │
│    (mypassword) WiFi Password            ◄── CONFIGURE THIS     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### Set WiFi SSID
**Action:**
1. Navigate to **(myssid) WiFi SSID**
2. Press **Enter**
3. Delete default text (use Backspace)
4. Type your network name (e.g., **MyHomeWiFi**)
5. Press **Enter**

#### Set WiFi Password
**Action:**
1. Navigate to **(mypassword) WiFi Password**
2. Press **Enter**
3. Delete default text
4. Type your WiFi password
5. Press **Enter**

**Security Note:** The password is stored in plaintext in `sdkconfig`. For production, use encrypted NVS storage.

---

## Part 3: Optional Advanced Settings

### Logging Level (for Debugging)

If you want detailed debug logs:

From main menu:
1. Navigate to **Component config**
2. Navigate to **Log output**
3. Set **Default log verbosity** to **Debug** or **Verbose**

Or enable per-component:
1. **Component config** → **Ethernet** → **Set Log Level** → **Debug**
2. **Component config** → **Wi-Fi** → **Set Log Level** → **Debug**

### LWIP Network Stack

From main menu:
1. **Component config** → **LWIP**
2. Enable **IP forwarding** if you want routing between WiFi and Ethernet
3. Adjust **TCPIP task priority** (default 19 is usually fine)

---

## Part 4: Save and Exit

### Step 14: Save Configuration

**Action:**
1. Press **S** (Save)
2. You'll see: "Configuration saved to /path/to/sdkconfig"
3. Press **Enter**

### Step 15: Exit Menuconfig

**Action:**
1. Press **Esc** repeatedly until asked to exit
2. Or press **Q** (Quit)
3. If prompted to save changes, press **Y** (Yes)

---

## Verification

After exiting menuconfig, verify your settings:

```bash
# Check that Ethernet is enabled
grep CONFIG_ETH_ENABLED sdkconfig

# Should show:
# CONFIG_ETH_ENABLED=y

# Check GPIO pins
grep CONFIG_ETH_SPI sdkconfig

# Should show:
# CONFIG_ETH_SPI_SCLK_GPIO=13
# CONFIG_ETH_SPI_MOSI_GPIO=11
# CONFIG_ETH_SPI_MISO_GPIO=12
# CONFIG_ETH_SPI_CS_GPIO=14
# ... etc
```

---

## Complete Configuration Summary

After completing all steps, your configuration should have:

### Ethernet Settings
- **Support Ethernet:** Enabled ✓
- **Ethernet Type:** SPI Ethernet MAC ✓
- **W5500 Support:** Enabled ✓
- **SPI Host:** SPI2_HOST (HSPI) ✓
- **SCLK GPIO:** 13 ✓
- **MOSI GPIO:** 11 ✓
- **MISO GPIO:** 12 ✓
- **CS GPIO:** 14 ✓
- **INT GPIO:** 10 ✓
- **Clock Speed:** 20 MHz ✓
- **PHY Address:** 1 ✓
- **PHY Reset GPIO:** 9 ✓

### WiFi Settings
- **SSID:** Your network name ✓
- **Password:** Your password ✓

---

## Troubleshooting Menuconfig

### "Option not available" or greyed out

**Cause:** Parent option not enabled  
**Solution:** Enable the parent option first (e.g., "Support Ethernet" must be enabled before SPI options appear)

### Can't find "Ethernet" menu

**Cause:** Wrong ESP-IDF version or target not set  
**Solution:**
```bash
# Set target to ESP32-S3
idf.py set-target esp32s3

# Then run menuconfig again
idf.py menuconfig
```

### Changes don't take effect

**Cause:** Not saved properly  
**Solution:** Always press **S** to save before exiting

### sdkconfig keeps resetting

**Cause:** `sdkconfig.defaults` overriding manual settings  
**Solution:** Either edit `sdkconfig.defaults` directly, or delete it before running menuconfig

---

## Next Steps

After configuration:

```bash
# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor
idf.py -p /dev/ttyUSB0 monitor
```

---

## Quick Reference Card

Print this for easy reference while configuring:

```
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 W5500 Ethernet Configuration Quick Reference     │
├─────────────────────────────────────────────────────────────┤
│  Navigation:                                                │
│    ↑↓ arrows    - Move                                      │
│    Enter        - Select                                    │
│    Y/Spacebar   - Enable                                    │
│    N            - Disable                                   │
│    S            - Save                                      │
│    Q/Esc Esc    - Exit                                      │
├─────────────────────────────────────────────────────────────┤
│  GPIO Pins (ESP32-S3 to W5500):                             │
│    SCLK  → GPIO 13                                          │
│    MOSI  → GPIO 11                                          │
│    MISO  → GPIO 12                                          │
│    CS    → GPIO 14                                          │
│    INT   → GPIO 10                                          │
│    RST   → GPIO 9                                           │
├─────────────────────────────────────────────────────────────┤
│  Menu Path:                                                 │
│    Component config → Ethernet                              │
│      [*] Support Ethernet                                   │
│      → Ethernet Type → SPI Ethernet MAC                     │
│      → SPI Ethernet Configuration                           │
│        [*] Use W5500 (MAC/PHY)                              │
│        → Ethernet SPI Host → SPI2_HOST                      │
│        Configure all GPIO pins                              │
│        Set clock speed: 20 MHz                              │
│        PHY Address: 1                                       │
│        PHY Reset GPIO: 9                                    │
├─────────────────────────────────────────────────────────────┤
│  WiFi Path:                                                 │
│    WiFi Configuration (top level)                           │
│      Set SSID                                               │
│      Set Password                                           │
└─────────────────────────────────────────────────────────────┘
```

---

**Questions?** Refer to the main README.md for troubleshooting tips!
