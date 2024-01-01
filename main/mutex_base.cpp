/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */

#include "mutex_base.h"

#include <esp_err.h>
#include <esp_log.h>

#include "common.h"

MutexBase::MutexBase(void) {
    // TODO: Use static version?
    this->mutex = xSemaphoreCreateMutex();
    if (this->mutex == NULL) {
        ESP_LOGE("mutex_base", "Failed to create mutex");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
}

MutexBase::MutexBase(const TickType_t wait_time) {
    this->wait_time = wait_time;
    // TODO: Use static version?
    this->mutex = xSemaphoreCreateMutex();
    if (this->mutex == NULL) {
        ESP_LOGE("mutex_base", "Failed to create mutex");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
}

bool MutexBase::take_mutex(const char *const tag, const char *const func) {
    bool result = true;
    if (xSemaphoreTake(this->mutex, this->wait_time) != pdTRUE) {
        // TODO: Log task name too?
        ESP_LOGE(tag, "Failed to take mutex in function %s", func);
        result = false;
    }
    return result;
}

void MutexBase::release_mutex(const char *const tag, const char *const func) {
    if (xSemaphoreGive(this->mutex) != pdTRUE) {
        // TODO: Log task name too?
        ESP_LOGE(tag, "Failed to release the mutex in function %s", func);
    }
}