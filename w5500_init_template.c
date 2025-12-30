/**
 * @file w5500_init_template.c
 * @brief Complete W5500 Ethernet Initialization Template
 * 
 * This template includes ALL critical configuration required for W5500.
 * Tested on ESP32-S3 with Waveshare ESP32-S3-ETH and external W5500 modules.
 * 
 * CRITICAL REQUIREMENTS:
 * 1. SPI device MUST have .command_bits = 16 and .address_bits = 8
 * 2. MAC config MUST NOT be NULL
 * 3. Kconfig MUST use 'select' to enable ESP-IDF ETH component
 * 4. Always run: idf.py set-target esp32s3 BEFORE building
 * 
 * @author Ahmed Al-Alousi
 * @date December 2025
 */

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"

#ifdef CONFIG_ENABLE_ETHERNET
#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#endif

static const char *TAG = "W5500_INIT";

#ifdef CONFIG_ENABLE_ETHERNET
static esp_netif_t *s_eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
#endif

/**
 * @brief Initialize W5500 Ethernet interface
 * 
 * CRITICAL NOTES:
 * - This function contains ALL required configuration for W5500
 * - Do NOT simplify or remove seemingly "redundant" settings
 * - Each configuration has been validated through extensive testing
 * 
 * @return ESP_OK on success, ESP_FAIL if W5500 not responding
 */
esp_err_t w5500_init(void)
{
#ifdef CONFIG_ENABLE_ETHERNET
    ESP_LOGI(TAG, "Initialising W5500 Ethernet...");
    
    // =========================================================================
    // STEP 1: Create Network Interface
    // =========================================================================
    esp_netif_config_t eth_netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&eth_netif_cfg);
    if (s_eth_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create netif");
        return ESP_FAIL;
    }
    
    // =========================================================================
    // STEP 2: Initialize SPI Bus
    // =========================================================================
    ESP_LOGI(TAG, "Initialising SPI bus (Host=%d)...", CONFIG_ETH_SPI_HOST);
    
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = CONFIG_ETH_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_ETH_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    
    esp_err_t ret = spi_bus_initialize(CONFIG_ETH_SPI_HOST, &spi_bus_cfg, 
                                       SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✓ SPI bus initialised");
    
    // =========================================================================
    // STEP 3: Configure W5500 SPI Device
    // =========================================================================
    // CRITICAL: W5500 requires specific command and address bit configuration
    // DO NOT REMOVE these fields - they are MANDATORY for W5500 protocol
    
    ESP_LOGI(TAG, "Configuring W5500 SPI device...");
    ESP_LOGI(TAG, "  MOSI: GPIO %d", CONFIG_ETH_SPI_MOSI_GPIO);
    ESP_LOGI(TAG, "  MISO: GPIO %d", CONFIG_ETH_SPI_MISO_GPIO);
    ESP_LOGI(TAG, "  SCLK: GPIO %d", CONFIG_ETH_SPI_SCLK_GPIO);
    ESP_LOGI(TAG, "  CS:   GPIO %d", CONFIG_ETH_SPI_CS_GPIO);
    ESP_LOGI(TAG, "  INT:  GPIO %d", CONFIG_ETH_SPI_INT_GPIO);
    ESP_LOGI(TAG, "  RST:  GPIO %d", CONFIG_ETH_PHY_RST_GPIO);
    ESP_LOGI(TAG, "  Clock: %d MHz", CONFIG_ETH_SPI_CLOCK_MHZ);
    
    spi_device_interface_config_t spi_devcfg = {
        // =====================================================================
        // CRITICAL W5500 REQUIREMENTS
        // =====================================================================
        .command_bits = 16,    // W5500 protocol requires 16-bit command
        .address_bits = 8,     // W5500 protocol requires 8-bit address
        // =====================================================================
        
        .mode = 0,             // SPI mode 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = CONFIG_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .queue_size = 20,      // Transaction queue size
        .spics_io_num = CONFIG_ETH_SPI_CS_GPIO,
    };
    
    // =========================================================================
    // STEP 4: Configure W5500 MAC
    // =========================================================================
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(
        CONFIG_ETH_SPI_HOST, &spi_devcfg);
    
    // Interrupt GPIO configuration
    w5500_config.int_gpio_num = CONFIG_ETH_SPI_INT_GPIO;
    
    // Use interrupt mode (0) instead of polling
    // Set to non-zero (e.g., 100) to use polling mode with specified period in ms
    w5500_config.poll_period_ms = 0;
    
    // =========================================================================
    // STEP 5: Create MAC (CRITICAL: Cannot pass NULL for mac_config!)
    // =========================================================================
    // WRONG:  esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, NULL);
    // CORRECT: Pass valid eth_mac_config_t pointer
    
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    
    ESP_LOGI(TAG, "Creating W5500 MAC...");
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    
    if (mac == NULL) {
        ESP_LOGE(TAG, "✗ MAC creation FAILED - W5500 not responding!");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "Troubleshooting checklist:");
        ESP_LOGE(TAG, "  1. Check wiring (especially MOSI/MISO - easy to swap!)");
        ESP_LOGE(TAG, "  2. Verify power: W5500 needs stable 3.3V");
        ESP_LOGE(TAG, "  3. Check common ground between ESP32 and W5500");
        ESP_LOGE(TAG, "  4. Try lower SPI clock speed (10 MHz or 5 MHz)");
        ESP_LOGE(TAG, "  5. Verify GPIO numbers match your hardware");
        ESP_LOGE(TAG, "  6. Try different SPI host (SPI2_HOST vs SPI3_HOST)");
        ESP_LOGE(TAG, "");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✓ W5500 MAC created successfully");
    
    // =========================================================================
    // STEP 6: Create PHY
    // =========================================================================
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_ETH_PHY_RST_GPIO;
    
    ESP_LOGI(TAG, "Creating W5500 PHY (addr=%d)...", CONFIG_ETH_PHY_ADDR);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
    
    if (phy == NULL) {
        ESP_LOGE(TAG, "✗ PHY creation failed");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✓ W5500 PHY created successfully");
    
    // =========================================================================
    // STEP 7: Install Ethernet Driver
    // =========================================================================
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    
    ret = esp_eth_driver_install(&eth_config, &s_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet driver install failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✓ Ethernet driver installed");
    
    // =========================================================================
    // STEP 8: Attach to Network Interface
    // =========================================================================
    ret = esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Netif attach failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✓ Ethernet attached to netif");
    
    // =========================================================================
    // STEP 9: Register Event Handlers (Optional - add your handlers here)
    // =========================================================================
    // Example:
    // ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
    //                                            &eth_event_handler, NULL));
    // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
    //                                            &ip_event_handler, NULL));
    
    ESP_LOGI(TAG, "✓✓✓ W5500 Ethernet initialised successfully ✓✓✓");
    
    return ESP_OK;
    
#else
    ESP_LOGI(TAG, "Ethernet disabled (enable in menuconfig)");
    return ESP_OK;
#endif
}

/**
 * @brief Start W5500 Ethernet interface
 * 
 * Call this after w5500_init() to begin network operations.
 * 
 * @return ESP_OK on success
 */
esp_err_t w5500_start(void)
{
#ifdef CONFIG_ENABLE_ETHERNET
    ESP_LOGI(TAG, "Starting W5500 Ethernet...");
    
    esp_err_t ret = esp_eth_start(s_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Ethernet: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✓ W5500 Ethernet started (non-blocking)");
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

/**
 * @brief Get W5500 network interface handle
 * 
 * @return esp_netif_t* or NULL if not initialized
 */
esp_netif_t* w5500_get_netif(void)
{
#ifdef CONFIG_ENABLE_ETHERNET
    return s_eth_netif;
#else
    return NULL;
#endif
}

/**
 * @brief Get W5500 Ethernet handle
 * 
 * @return esp_eth_handle_t or NULL if not initialized
 */
esp_eth_handle_t w5500_get_handle(void)
{
#ifdef CONFIG_ENABLE_ETHERNET
    return s_eth_handle;
#else
    return NULL;
#endif
}

// =============================================================================
// KCONFIG.PROJBUILD TEMPLATE
// =============================================================================
/*

menu "Ethernet Configuration"
    config ENABLE_ETHERNET
        bool "Enable Ethernet (W5500)"
        default y
        select ETH_ENABLED              # CRITICAL: Enable ESP-IDF ETH component
        select ETH_USE_SPI_ETHERNET     # CRITICAL: Enable SPI Ethernet
        select ETH_SPI_ETHERNET_W5500   # CRITICAL: Enable W5500 driver
        help
            Enable W5500 Ethernet module for wired network connectivity.

    if ENABLE_ETHERNET
        config ETH_SPI_HOST
            int "SPI Host"
            default 2
            range 0 2
            help
                SPI peripheral to use for W5500 communication.
                SPI2_HOST = 2 (recommended for ESP32-S3)
                Try different values if W5500 doesn't respond.

        config ETH_SPI_SCLK_GPIO
            int "SPI SCLK GPIO"
            range 0 48
            default 13
            help
                GPIO pin for SPI clock line (SCK).

        config ETH_SPI_MOSI_GPIO
            int "SPI MOSI GPIO"
            range 0 48
            default 11
            help
                GPIO pin for SPI Master Out Slave In (MOSI).

        config ETH_SPI_MISO_GPIO
            int "SPI MISO GPIO"
            range 0 48
            default 12
            help
                GPIO pin for SPI Master In Slave Out (MISO).

        config ETH_SPI_CS_GPIO
            int "SPI CS GPIO"
            range 0 48
            default 14
            help
                GPIO pin for SPI Chip Select (CS).

        config ETH_SPI_INT_GPIO
            int "Interrupt GPIO"
            range -1 48
            default 10
            help
                GPIO pin for W5500 interrupt (optional, -1 to disable).

        config ETH_SPI_CLOCK_MHZ
            int "SPI Clock Speed (MHz)"
            default 20
            range 1 40
            help
                SPI clock frequency for W5500 communication.
                20MHz is recommended for stable operation.
                Try 10 or 5 MHz if experiencing issues.

        config ETH_PHY_RST_GPIO
            int "PHY Reset GPIO"
            range -1 48
            default 9
            help
                GPIO pin for W5500 reset (optional, -1 to disable).

        config ETH_PHY_ADDR
            int "PHY Address"
            default 1
            range 0 31
            help
                W5500 PHY address (usually 1, try 0 if not working).
    endif
endmenu

*/

// =============================================================================
// SDKCONFIG.DEFAULTS TEMPLATE
// =============================================================================
/*

# ESP32-S3 Target Configuration (CRITICAL)
CONFIG_IDF_TARGET="esp32s3"
CONFIG_IDF_TARGET_ESP32S3=y

# Flash Configuration (16MB for Waveshare ESP32-S3-ETH)
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="16MB"
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y

# PSRAM Configuration (8MB Octal PSRAM)
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_USE_MALLOC=y
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384
CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=32768
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y

# USB Serial/JTAG Console (ESP32-S3 native USB)
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
CONFIG_ESP_CONSOLE_SECONDARY_NONE=y

# Ethernet Configuration
# Controlled via Kconfig.projbuild (menuconfig: Ethernet Configuration)
CONFIG_ENABLE_ETHERNET=y

*/

// =============================================================================
// USAGE EXAMPLE
// =============================================================================
/*

void app_main(void)
{
    // Initialize NVS (required)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || 
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize W5500
    if (w5500_init() == ESP_OK) {
        // Start W5500
        w5500_start();
    }
    
    // Your application code here...
}

*/

// =============================================================================
// TROUBLESHOOTING GUIDE
// =============================================================================
/*

PROBLEM: "invalid argument" error, MAC creation fails
CAUSE: Missing .command_bits and .address_bits in SPI config, OR NULL mac_config
FIX: Ensure spi_devcfg has .command_bits=16 and .address_bits=8
     Ensure mac_config is valid pointer, not NULL

PROBLEM: "fatal error: esp_eth.h: No such file or directory"
CAUSE: Kconfig doesn't enable ESP-IDF ETH component
FIX: Add 'select ETH_ENABLED' to Kconfig.projbuild ENABLE_ETHERNET option

PROBLEM: Flash size mismatch, brownout on boot
CAUSE: Wrong target or old sdkconfig
FIX: Run: rm sdkconfig && idf.py set-target esp32s3 && idf.py build

PROBLEM: W5500 works on Waveshare but not on custom board
CAUSE: Different SPI host number
FIX: Try CONFIG_ETH_SPI_HOST = 1 instead of 2 (or vice versa)

PROBLEM: Random crashes or instability
CAUSE: Poor power supply or long wires causing signal integrity issues
FIX: 1) Check 3.3V is stable with multimeter
     2) Reduce SPI clock speed to 10 or 5 MHz
     3) Use shorter wires, twisted pairs for MOSI/MISO
     4) Add 0.1µF capacitor near W5500 VCC pin

*/
