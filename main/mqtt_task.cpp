/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief Implementation of the MQTTClient class

#include "mqtt_task.h"

#include <esp_crt_bundle.h>
#include <esp_err.h>
#include <esp_log.h>
#include <mqtt_client.h>
#include <string.h>

#define MQTT_LOG_TAG "mqtt_task"

/// @brief Max number of characters for a float message.
#define MAX_FLOAT_MESSAGE_LEN 44
/// @brief Max number of characters for BSEC JSON message
#define MAX_BSEC_JSON_MESSAGE_LEN 85

/// @brief Format string for sending float messages
#define FLOAT_FORMAT_STR "%0.6f"
/// @brief Format string for sending BSEC json message
#define BSEC_JSON_FORMAT_STR "{\"value\":%0.6f,\"accuracy\":%u,\"valid\":%s}"

// TODO: Support storing configuration in filesystem?
// See mqtt_task.h for documentation
MQTTClient::MQTTClient(const char *const uri, const char *const username,
                       const char *const password) {
    esp_mqtt_client_config_t mqtt_config;
    (void)memset(&mqtt_config, 0, sizeof(esp_mqtt_client_config_t));
    mqtt_config.broker.address.uri = uri;
    mqtt_config.credentials.username = username;
    mqtt_config.credentials.authentication.password = password;
    mqtt_config.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    this->client_handle = esp_mqtt_client_init(&mqtt_config);
    if (this->client_handle == NULL) {
        ESP_LOGE(MQTT_LOG_TAG, "Failed to init MQTT client for uri: %s", uri);
        ESP_ERROR_CHECK(ESP_FAIL);
    }
}

// See mqtt_task.h for documentation
esp_err_t MQTTClient::start(void) {
    return esp_mqtt_client_start(this->client_handle);
}

// See mqtt_task.h for documentation
esp_err_t MQTTClient::reconnect(void) {
    return esp_mqtt_client_reconnect(this->client_handle);
}

// See mqtt_task.h for documentation
esp_err_t MQTTClient::disconnect(void) {
    return esp_mqtt_client_reconnect(this->client_handle);
}

// See mqtt_task.h for documentation
esp_err_t MQTTClient::stop(void) {
    return esp_mqtt_client_stop(this->client_handle);
}

// See mqtt_task.h for documentation
int MQTTClient::publish(const char *const topic, const char *const data,
                        int len, int qos, int retain) {
    return esp_mqtt_client_publish(this->client_handle, topic, data, len, qos,
                                   retain);
}

// See mqtt_task.h for documentation
int MQTTClient::publish(const char *const topic, float data, int qos,
                        int retain) {
    char message[MAX_FLOAT_MESSAGE_LEN + 1] = "";
    (void)snprintf(message, MAX_FLOAT_MESSAGE_LEN, FLOAT_FORMAT_STR, data);
    return this->publish(topic, message, MAX_FLOAT_MESSAGE_LEN + 1, qos,
                         retain);
}

// See mqtt_task.h for documentation
int MQTTClient::publish(const char *const topic,
                        bsec_virtual_sensor_data_t data, int qos, int retain) {
    char message[MAX_BSEC_JSON_MESSAGE_LEN + 1] = "";
    (void)snprintf(message, MAX_BSEC_JSON_MESSAGE_LEN, BSEC_JSON_FORMAT_STR,
                   data.signal, data.accuracy, data.valid ? "true" : "false");
    return this->publish(topic, message, MAX_BSEC_JSON_MESSAGE_LEN + 1, qos,
                         retain);
}

// See mqtt_task.h for documentation
int MQTTClient::enqueue(const char *const topic, const char *const data,
                        int len, int qos, int retain, bool store) {
    return esp_mqtt_client_enqueue(this->client_handle, topic, data, len, qos,
                                   retain, store);
}

// See mqtt_task.h for documentation
int MQTTClient::enqueue(const char *const topic, float data, int qos,
                        int retain, bool store) {
    char message[MAX_FLOAT_MESSAGE_LEN + 1] = "";
    (void)snprintf(message, MAX_FLOAT_MESSAGE_LEN, FLOAT_FORMAT_STR, data);
    return this->enqueue(topic, message, MAX_FLOAT_MESSAGE_LEN + 1, qos, retain,
                         store);
}

// See mqtt_task.h for documentation
int MQTTClient::enqueue(const char *const topic,
                        bsec_virtual_sensor_data_t data, int qos, int retain,
                        bool store) {
    char message[MAX_BSEC_JSON_MESSAGE_LEN + 1] = "";
    (void)snprintf(message, MAX_BSEC_JSON_MESSAGE_LEN, BSEC_JSON_FORMAT_STR,
                   data.signal, data.accuracy, data.valid ? "true" : "false");
    return this->enqueue(topic, message, MAX_FLOAT_MESSAGE_LEN + 1, qos, retain,
                         store);
}