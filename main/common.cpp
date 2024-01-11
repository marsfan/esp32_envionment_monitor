/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include "common.h"

#include <esp_err.h>
#include <esp_log.h>
#include <sys/time.h>

// See common.h for docs
int64_t get_epoch_time_s(void) {
    return get_epoch_time_us() / S_IN_US;
}

// See common.h for docs
int64_t get_epoch_time_ms(void) {
    return get_epoch_time_us() / US_IN_MS;
}

// See common.h for docs
int64_t get_epoch_time_us(void) {
    struct timeval timeval = {.tv_sec = 0, .tv_usec = 0};
    gettimeofday(&timeval, NULL);
    return (timeval.tv_sec * S_IN_US) + (timeval.tv_usec);
}

// See common.h for docs
int64_t get_epoch_time_ns(void) {
    return get_epoch_time_us() * NS_IN_US;
}