/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */

#include "wifi_network.h"

#include <esp_log.h>
#include <esp_netif_sntp.h>
#include <string.h>

#include "common.h"

#define WIFI_LOG_TAG "wifi_network"

// Max number of found wifi networks. Needs to be small(ish) to save stack.
#define MAX_FOUND_NETWORKS 10

/// @brief Max number of times to retry connecting
#define MAX_CONNECTION_RETRY 5

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT 0b01
#define WIFI_FAIL_BIT 0b10

/// @brief NTP Server to use to get time
#define NTP_SERVER "pool.ntp.org"

/// @brief Whether or not to smoothly adjust system time
#define USE_SMOOTH_SYNC true

// static void ip_event_handler(void* arg, int32_t event_id, void* event_data);
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data);
// See wifi_network.h for documentation.
WiFiNetwork::WiFiNetwork(void) {
    this->init_config = WIFI_INIT_CONFIG_DEFAULT();

    // TODO: Enable NVS Support
    this->init_config.nvs_enable = false;

    (void)memset(&this->wifi_config, 0, sizeof(wifi_config_t));

    this->connection_retry_count = 0;
}

// See wifi_network.h for documentation.
esp_err_t WiFiNetwork::init(void) {
    esp_err_t result = ESP_OK;

    // Create WiFi event group
    this->wifi_event_group = xEventGroupCreate();

    // Init network interface
    result = esp_netif_init();
    LOGE_ON_ERROR(WIFI_LOG_TAG, __func__,
                  "Failed Initializing Network Interface", result);

    // Create event loop for handling
    if (result == ESP_OK) {
        result = esp_event_loop_create_default();
        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__,
                      "Failed creating default event loop", result);
    }

    // Initialize WiFi
    if (result == ESP_OK) {
        // Init WiFi system as a station
        this->netif = esp_netif_create_default_wifi_sta();

        result = esp_wifi_init(&this->init_config);
        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__, "Failed Initializing WiFi",
                      result);
    }

    // Set WiFi mode to Station
    // TODO: Support AP+STA mode?
    if (result == ESP_OK) {
        result = esp_wifi_set_mode(WIFI_MODE_STA);
        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__, "Failed Setting WiFi Mode",
                      result);
    }

    // Set event callback for WiFi events
    if (result == ESP_OK) {
        result = esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, this,
            &this->wifi_event_instance_any_id);
        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__,
                      "Failed registering WiFi event handler", result);
    }

    // Set event callback for IP Events
    if (result == ESP_OK) {
        result = esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, this,
            &this->ip_sta_got_ip_event_instance);
        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__,
                      "Failed Registering IP Event handler", result);
    }

    // Start WiFi
    if (result == ESP_OK) {
        result = esp_wifi_start();
        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__, "Failed Staring WiFi", result);
    }

    return result;
}

// TODO: Support returning the scanned networks instead of printing them?
// TODO: Argument for blocking?
// See wifi_network.h for docs.
esp_err_t WiFiNetwork::scan_for_networks(const wifi_scan_config_t* config) {
    esp_err_t result = ESP_OK;
    uint16_t num_found = 0;
    wifi_ap_record_t found_networks[MAX_FOUND_NETWORKS];
    (void)memset(&found_networks, 0, sizeof(found_networks));

    // Start the scan
    result = esp_wifi_scan_start(config, true);
    LOGE_ON_ERROR(WIFI_LOG_TAG, __func__, "Failed Starting WiFi Scan", result);

    // Get number of found networks
    if (result == ESP_OK) {
        result = esp_wifi_scan_get_ap_num(&num_found);
        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__,
                      "Failed to get number of found networks", result);
    }

    // Get the found networks
    if (result == ESP_OK) {
        result = esp_wifi_scan_get_ap_records(&num_found, found_networks);
        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__,
                      "Failed to get the found networks", result);
    }

    // Print found networks
    if (result == ESP_OK) {
        ESP_LOGI(WIFI_LOG_TAG, "Found %d WiFi Networks: ", num_found);
        for (int i = 0; i < num_found; i++) {
            // TODO: More info
            // TODO: Auth Mode to string?
            ESP_LOGI(WIFI_LOG_TAG, "%d: SSID='%s', RSSI=%d, Auth Mode: %d", i,
                     found_networks[i].ssid, found_networks[i].rssi,
                     found_networks[i].authmode);
        }
    }

    return result;
}

// See wifi_network.h for docs.
esp_err_t WiFiNetwork::scan_for_networks(void) {
    return this->scan_for_networks(NULL);
}

// See wifi_network.h for docs
esp_err_t WiFiNetwork::connect_to_ap(const char* const ssid,
                                     const char* const password) {
    esp_err_t result = ESP_OK;

    // Copy SSID and Password to config structure
    (void)strncpy((char*)this->wifi_config.sta.ssid, ssid,
                  sizeof(wifi_config.sta.ssid));
    (void)strncpy((char*)this->wifi_config.sta.password, password,
                  sizeof(wifi_config.sta.ssid));

    result = esp_wifi_set_config(WIFI_IF_STA, &this->wifi_config);
    LOGE_ON_ERROR(WIFI_LOG_TAG, __func__, "Failed Setting WiFi COnfig", result);

    // Actually try to connect.
    if (result == ESP_OK) {
        result = esp_wifi_connect();
        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__, "Failed Connecting to Network",
                      result);
    }

    // Wait for connect to pass/fail
    if (result == ESP_OK) {
        EventBits_t bits = xEventGroupWaitBits(
            this->wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE,
            pdFALSE, portMAX_DELAY);
        if ((bits & WIFI_FAIL_BIT) != 0) {
            result = ESP_ERR_TIMEOUT;
        }
    }

    // Initialize NTP system
    if (result == ESP_OK) {
        esp_sntp_config sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG(NTP_SERVER);
        sntp_config.smooth_sync = USE_SMOOTH_SYNC;
        result = esp_netif_sntp_init(&sntp_config);

        LOGE_ON_ERROR(WIFI_LOG_TAG, __func__, "Failed setting up SNTP", result);
    } else {
        ESP_LOGI(WIFI_LOG_TAG, "Started NTP system");
    }
    return result;
}

// See wifi_network.h for docs
esp_err_t WiFiNetwork::disconnect(void) {
    esp_netif_sntp_deinit();
    return esp_wifi_disconnect();
}

// See wifi_network.h for docs
esp_err_t WiFiNetwork::update_time_from_network(void) {
    return this->update_time_from_network(10000);
}

// See wifi_network.h for docs
esp_err_t WiFiNetwork::update_time_from_network(uint32_t wait_time) {
    return esp_netif_sntp_sync_wait(pdMS_TO_TICKS(wait_time));
}

// See wifi_network.h for docs
void WiFiNetwork::wifi_event_handler(void* arg, int32_t event_id,
                                     void* event_data) {
    switch (event_id) {
        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(WIFI_LOG_TAG, "Finished Scanning for Networks");
            break;
        case WIFI_EVENT_STA_START:
            ESP_LOGI(WIFI_LOG_TAG, "WiFi STA Started.");
            // esp_wifi_connect();

            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(WIFI_LOG_TAG, "WiFi Station Connected.");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(WIFI_LOG_TAG, "WiFi Station Disconnected");
            if (this->connection_retry_count > MAX_CONNECTION_RETRY) {
                esp_wifi_connect();
                ESP_LOGI(WIFI_LOG_TAG, "Retrying Connection. #%d",
                         this->connection_retry_count);
                this->connection_retry_count++;
            } else {
                xEventGroupSetBits(this->wifi_event_group, WIFI_FAIL_BIT);
            }
            break;
        default:
            ESP_LOGE(WIFI_LOG_TAG, "Unknown WiFi Event: %lu", event_id);
            break;
    }
}

// See wifi_network.h for docs
void WiFiNetwork::ip_event_handler(void* arg, int32_t event_id,
                                   void* event_data) {
    switch (event_id) {
        // TODO: Add case for disconnect. Needs to be part of class so we
        // can have the counter
        case IP_EVENT_STA_GOT_IP: {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            if (event->ip_changed) {
                ESP_LOGI(WIFI_LOG_TAG,
                         "IPv4 DHCP Changed. Address=%d.%d.%d.%d, "
                         "Gateway=%d.%d.%d.%d, Netmask=%d.%d.%d.%d",
                         IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.gw),
                         IP2STR(&event->ip_info.netmask));
            } else {
                ESP_LOGI(WIFI_LOG_TAG,
                         "IPv4 DHCP Configuration complete. "
                         "Address=%d.%d.%d.%d, "
                         "Gateway=%d.%d.%d.%d, Netmask=%d.%d.%d.%d",
                         IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.gw),
                         IP2STR(&event->ip_info.netmask));
            }
            xEventGroupSetBits(this->wifi_event_group, WIFI_CONNECTED_BIT);
            break;
        }
        default:
            ESP_LOGE(WIFI_LOG_TAG, "Unknown IP Event: %lu", event_id);
            break;
    }
}

/// @brief Handle system events
/// @param arg Argument for the given event. Should be a pointer to the
/// WiFiNetwork instance
/// @param event_base Base event type
/// @param event_id ID of the event
/// @param event_data Data for the event
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    WiFiNetwork* wifi = (WiFiNetwork*)arg;
    if (event_base == WIFI_EVENT) {
        wifi->wifi_event_handler(arg, event_id, event_data);
    } else if (event_base == IP_EVENT) {
        wifi->ip_event_handler(arg, event_id, event_data);
    } else {
        ESP_LOGE("event_handler", "Unhandled event. base: %s, ID: %ld",
                 event_base, event_id);
    }
}