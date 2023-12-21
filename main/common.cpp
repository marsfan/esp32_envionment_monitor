/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include "common.h"

#include <esp_err.h>
#include <esp_log.h>

// See common.h for documentation
void log_e_on_error(esp_err_t err, const char *const tag,
                    const char *const func, const char *const msg) {
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Error | %s | %d | %s", func, err, msg);
    }
}