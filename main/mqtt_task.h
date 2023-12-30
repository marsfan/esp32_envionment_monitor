/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief Logic for the tasks that handle periodically publishing data to MQTT
/// brokers
#ifndef MQTT_TASK_H
#define MQTT_TASK_H

#include <esp_err.h>
#include <mqtt_client.h>

#include "bsec.h"  // bsec_virtual_sensor_data_t

/// @brief A single MQTT client class for sending data to a specific broker
class MQTTClient {
   public:
    /// @brief Instantiate a MQTT Client class
    /// @param uri URL of the broker to publish the data to
    /// @param username Username to use to connect to the broker
    /// @param password Password to use to connect to the broker.
    MQTTClient(const char *const uri, const char *const username,
               const char *const password);

    /// @brief Start MQTT client, connect to broker.
    /// @return Result of starting the client.
    esp_err_t start(void);

    /// @brief Reconnect to the broker
    /// @return Result of reconnecting the client to the broker.
    esp_err_t reconnect(void);

    /// @brief Disconnect from the broker
    /// @return Result of disconnecting the client from the broker.
    esp_err_t disconnect(void);

    /// @brief Stop the client.
    /// @return Result of stopping the client
    esp_err_t stop(void);

    /// @brief Publish a message to the broker
    /// @param topic The topic to publish
    /// @param data The data to publish to the topic
    /// @param len Length of the data string to publish
    /// @param qos The QOS level to use when publishing
    /// @param retain The value of the retain flag.
    /// @return Message ID of the publish message, or an error code.
    /// @retval >= 0: Success
    /// @retval -1: Failure
    /// @retval -2: Full outbox
    int publish(const char *const topic, const char *const data, int len,
                int qos, int retain);

    /// @brief Publish a float to the broker
    /// @details This will publish the data with an up to 44 character string. 6
    /// decimal precision. For other resolution, please manually create a string
    /// and publish the string instead.
    /// @param topic The topic to publish to
    /// @param data The data to publish
    /// @param qos The QOS level to use when publishing
    /// @param retain The value of the retain flag.
    /// @return Message ID of the publish message, or an error code.
    /// @retval >= 0: Success
    /// @retval -1: Failure
    /// @retval -2: Full outbox
    int publish(const char *const topic, float data, int qos, int retain);

    /// @brief Publish data from a BSEC virtual sensor to the broker
    /// @details This will publish the signal with an up to 44 character string.
    /// 6 decimal precision. For other resolution, please manually create a
    /// string and publish the string instead.
    /// @details The accuracy member will be  published as an integer, and the
    /// validity member will be published as a boolean.
    /// @details The data is published using JSON data.
    /// @param topic The topic to publish to
    /// @param data The data to publish
    /// @param qos The QOS level to use when publishing
    /// @param retain The value of the retain flag.
    /// @return Message ID of the publish message, or an error code.
    /// @retval >= 0: Success
    /// @retval -1: Failure
    /// @retval -2: Full outbox
    int publish(const char *const topic, bsec_virtual_sensor_data_t data,
                int qos, int retain);

    /// @brief Enqueue a message to be sent later.
    /// @param topic The topic to publish
    /// @param data The data to publish to the topic
    /// @param len Length of the data string to publish
    /// @param qos The QOS level to use when publishing
    /// @param retain The value of the retain flag.
    /// @param store If true, all messages are enqueued, otherwise only QOS 1
    /// and QOS 2 messages are enqueued.
    /// @return Message ID of the publish message, or an error code.
    /// @retval >= 0: Success
    /// @retval -1: Failure
    /// @retval -2: Full outbox
    int enqueue(const char *const topic, const char *const data, int len,
                int qos, int retain, bool store);

    /// @brief Enqueue a float to be sent later.
    /// @details This will publish the data with an up to 44 character string
    /// with 6 decimal precision. For other resolution please manually create a
    /// string and publish the string instead.
    /// @param topic The topic to publish to
    /// @param data The data to publish
    /// @param qos The QOS level to use when publishing
    /// @param retain The value of the retain flag.
    /// @param store If true, all messages are enqueued, otherwise only QOS 1
    /// and QOS 2 messages are enqueued.
    /// @return Message ID of the publish message, or an error code.
    /// @retval >= 0: Success
    /// @retval -1: Failure
    /// @retval -2: Full outbox
    int enqueue(const char *const topic, float data, int qos, int retain,
                bool store);

    /// @brief Enqueue data from a BSEC virtual sensor to the broker
    /// @details This will publish the signal with an up to 44 character string.
    /// 6 decimal precision. For other resolution, please manually create a
    /// string and publish the string instead.
    /// @details The accuracy member will be  published as an integer, and the
    /// validity member will be published as a boolean.
    /// @details The data is published using JSON data.
    /// @param topic The topic to publish to
    /// @param data The data to publish
    /// @param qos The QOS level to use when publishing
    /// @param retain The value of the retain flag.
    /// @param store If true, all messages are enqueued, otherwise only QOS 1
    /// and QOS 2 messages are enqueued.
    /// @return Message ID of the publish message, or an error code.
    /// @retval >= 0: Success
    /// @retval -1: Failure
    /// @retval -2: Full outbox
    int enqueue(const char *const topic, bsec_virtual_sensor_data_t data,
                int qos, int retain, bool store);

   private:
    /// @brief Handle of the MQTT client.
    esp_mqtt_client_handle_t client_handle;
};
#endif  // MQTT_TASK_H