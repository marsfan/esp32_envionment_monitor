/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief WiFi Network functionality

#ifndef WIFI_NETWORK_H
#define WIFI_NETWORK_H

#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/event_groups.h>

#include "wifi_config.h"

// TODO: Support station mode for setting up WiFi connection?
// TODO: Use singleton to ensure this is only instantiated once?
// TODO: Support other config options?
// TODO: NVS Support.

class WiFiNetwork {
   public:
    /// @brief Create an instance of the class.
    WiFiNetwork(void);

    /// @brief Initialize the WiFi system
    /// @return Result of initialization.
    esp_err_t init(void);

    /// @brief Shut down WiFi and lWiP system.
    /// @return Result of deinitialization.
    esp_err_t deinit(void);

    /// @brief Scan for WiFi networks
    /// @param config WiFi scan configuration. Set to NULL to use defaults.
    /// @return Result of the network scan.
    esp_err_t scan_for_networks(const wifi_scan_config_t *config);

    /// @brief Scan for wifi networks
    /// @return Result of the network scan
    esp_err_t scan_for_networks(void);

    /// @brief Connect to the given access point
    /// @param ssid SSID of the access point. Max 32 characters (inc null
    /// terminator)
    /// @param password Password of the access point. Max 64 characters (inc
    /// null terminator
    /// @return Result of connecting to the AP.
    esp_err_t connect_to_ap(const char *const ssid, const char *const password);

    /// @brief Disconnect from the currently connected AP
    /// @return Result of disconnecting from the AP
    esp_err_t disconnect(void);

    /// @brief Update the current time from the network
    /// @details Will wait up to 10 seconds for a response.
    /// @return Result of updating the time from the network
    esp_err_t update_time_from_network(void);

    /// @brief Update the current time from the network
    /// @param wait_time The maximum amount of time (in ms) to wait for the time
    /// to be updated
    /// @return Result of updating the time from the network
    esp_err_t update_time_from_network(uint32_t wait_time);

    /// @brief Handle WiFi Events
    /// @param arg Event argument
    /// @param event_id The event ID
    /// @param event_data Event data
    void wifi_event_handler(void *arg, int32_t event_id, void *event_data);

    /// @brief Handle IP Events
    /// @param arg Event argument
    /// @param event_id The event ID
    /// @param event_data Event data
    void ip_event_handler(void *arg, int32_t event_id, void *event_data);

   private:
    /// @brief Initialization config for the WiFi system
    wifi_init_config_t init_config;

    /// @brief Main WiFi Configuration structure.
    wifi_config_t wifi_config;

    /// @brief Event handler instants  for any WiFi event.
    esp_event_handler_instance_t wifi_event_instance_any_id;

    /// @brief Event handler instance for the IP_EVENT_STA_GOT_IP event.
    esp_event_handler_instance_t ip_sta_got_ip_event_instance;

    /// @brief Current number of retries for connecting to a network.
    uint8_t connection_retry_count;

    /// @brief Pointer to the network interface structure.
    esp_netif_t *netif;

    /// @brief Event group for signaling when WiFi is connected.
    EventGroupHandle_t wifi_event_group;
};

#endif  // NETWORK_H