/**
 * @file network.c
 * @brief Simple dual-interface network implementation (WiFi + W5500 Ethernet)
 * 
 * Standalone version for community example - no external dependencies
 */

#include "network.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <string.h>

#ifdef CONFIG_ENABLE_ETHERNET
#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#endif

static const char *TAG = "NETWORK";

// Network interface handles
static esp_netif_t *s_wifi_netif = NULL;
static bool s_wifi_connected = false;
static char s_wifi_ip[16] = "0.0.0.0";

#ifdef CONFIG_ENABLE_ETHERNET
static esp_netif_t *s_eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
static bool s_eth_connected = false;
static char s_eth_ip[16] = "0.0.0.0";
#endif

// ============================================================================
// Event Handlers
// ============================================================================

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi station started");
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WiFi connected to AP");
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
                s_wifi_connected = false;
                strcpy(s_wifi_ip, "0.0.0.0");
                esp_wifi_connect();
                break;
                
            default:
                break;
        }
    }
}

static void ip_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            snprintf(s_wifi_ip, sizeof(s_wifi_ip), IPSTR, 
                     IP2STR(&event->ip_info.ip));
            s_wifi_connected = true;
            ESP_LOGI(TAG, "✓ WiFi got IP: %s", s_wifi_ip);
        }
#ifdef CONFIG_ENABLE_ETHERNET
        else if (event_id == IP_EVENT_ETH_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            snprintf(s_eth_ip, sizeof(s_eth_ip), IPSTR, 
                     IP2STR(&event->ip_info.ip));
            s_eth_connected = true;
            ESP_LOGI(TAG, "✓ Ethernet got IP: %s", s_eth_ip);
        }
#endif
    }
}

#ifdef CONFIG_ENABLE_ETHERNET
static void eth_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == ETH_EVENT) {
        switch (event_id) {
            case ETHERNET_EVENT_CONNECTED:
                ESP_LOGI(TAG, "Ethernet link up");
                break;
                
            case ETHERNET_EVENT_DISCONNECTED:
                ESP_LOGW(TAG, "Ethernet link down");
                s_eth_connected = false;
                strcpy(s_eth_ip, "0.0.0.0");
                break;
                
            case ETHERNET_EVENT_START:
                ESP_LOGI(TAG, "Ethernet started");
                break;
                
            case ETHERNET_EVENT_STOP:
                ESP_LOGI(TAG, "Ethernet stopped");
                break;
                
            default:
                break;
        }
    }
}
#endif

// ============================================================================
// Public API Implementation
// ============================================================================

esp_err_t network_init(void)
{
    ESP_LOGI(TAG, "Initialising dual-interface network...");
    
    // Initialise TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // =========================================================================
    // WiFi Initialisation
    // =========================================================================
    
    s_wifi_netif = esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    
    // Register WiFi event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &ip_event_handler, NULL));
    
    // Configure WiFi (credentials from menuconfig)
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    ESP_LOGI(TAG, "✓ WiFi initialised (SSID: %s)", CONFIG_WIFI_SSID);
    
#ifdef CONFIG_ENABLE_ETHERNET
    // =========================================================================
    // Ethernet Initialisation (W5500)
    // =========================================================================
    
    // Create Ethernet netif
    esp_netif_config_t eth_netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&eth_netif_cfg);
    
    // Configure SPI bus for W5500
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = CONFIG_ETH_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_ETH_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_ETH_SPI_HOST, &spi_bus_cfg,
                                       SPI_DMA_CH_AUTO));
    
    // Configure W5500 SPI device
    // Configure W5500 SPI device (CRITICAL: W5500 requires specific bit configuration)
    spi_device_interface_config_t spi_devcfg = {
        .command_bits = 16,        // W5500 requires 16-bit command
        .address_bits = 8,         // W5500 requires 8-bit address
        .mode = 0,
        .clock_speed_hz = CONFIG_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .queue_size = 20,
        .spics_io_num = CONFIG_ETH_SPI_CS_GPIO,
    };
    
    // W5500 MAC configuration
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(
        CONFIG_ETH_SPI_HOST, &spi_devcfg);
    w5500_config.int_gpio_num = CONFIG_ETH_SPI_INT_GPIO;
    w5500_config.poll_period_ms = 0;  // Use interrupt mode
    
    // MAC config (REQUIRED - cannot be NULL!)
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    
    // Create MAC
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    
    // Create PHY
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_ETH_PHY_RST_GPIO;
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
    
    // Configure Ethernet driver
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));
    
    // Attach Ethernet driver to netif
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, 
                    esp_eth_new_netif_glue(s_eth_handle)));
    
    // Register Ethernet event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &ip_event_handler, NULL));
    
    ESP_LOGI(TAG, "✓ Ethernet (W5500) initialised");
#else
    ESP_LOGI(TAG, "Ethernet disabled (enable in menuconfig)");
#endif
    
    ESP_LOGI(TAG, "✓ Dual-interface network initialised");
    return ESP_OK;
}

esp_err_t network_start(void)
{
    ESP_LOGI(TAG, "Starting network connections...");
    
    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    
#ifdef CONFIG_ENABLE_ETHERNET
    // Start Ethernet
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
#endif
    
    ESP_LOGI(TAG, "✓ Network connections started (non-blocking)");
    return ESP_OK;
}

bool network_is_connected(network_interface_t interface)
{
    switch (interface) {
        case NETWORK_IF_WIFI:
            return s_wifi_connected;
#ifdef CONFIG_ENABLE_ETHERNET
        case NETWORK_IF_ETHERNET:
            return s_eth_connected;
#endif
        default:
            return false;
    }
}

const char* network_get_ip(network_interface_t interface)
{
    switch (interface) {
        case NETWORK_IF_WIFI:
            return s_wifi_connected ? s_wifi_ip : NULL;
#ifdef CONFIG_ENABLE_ETHERNET
        case NETWORK_IF_ETHERNET:
            return s_eth_connected ? s_eth_ip : NULL;
#endif
        default:
            return NULL;
    }
}

network_interface_t network_get_active_interface(void)
{
#ifdef CONFIG_ENABLE_ETHERNET
    // Prefer Ethernet if both connected
    if (s_eth_connected) {
        return NETWORK_IF_ETHERNET;
    }
#endif
    // Default to WiFi
    return NETWORK_IF_WIFI;
}
