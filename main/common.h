/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief Common functionality to use across modules.

#ifndef COMMON_H
#define COMMON_H

#include <esp_err.h>
#include <esp_log.h>

#define LOGE_ON_ERROR(tag, func, msg, err)                                \
    if (err != ESP_OK) {                                                  \
        ESP_LOGE(tag, "Error | %s | %s | %s", func, esp_err_to_name(err), \
                 msg);                                                    \
    }

/// @brief Get the epoch time in seconds
/// @return The epoch time in seconds
int64_t get_epoch_time_s(void);

/// @brief Get the epoch time in MS
/// @return The epoch time in ms
int64_t get_epoch_time_ms(void);

/// @brief Get the epoch time in us
/// @return The epoch time in us
int64_t get_epoch_time_us(void);

/// @brief Get the epoch time in ns
/// @return The epoch time in ns
int64_t get_epoch_time_ns(void);

#endif  // COMMON_H