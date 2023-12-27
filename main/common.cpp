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
    return get_epoch_time_us() / 1000000;
}

// See common.h for docs
int64_t get_epoch_time_ms(void) {
    return get_epoch_time_us() / 1000;
}

// See common.h for docs
int64_t get_epoch_time_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000000) + (tv.tv_usec);
}

// See common.h for docs
int64_t get_epoch_time_ns(void) {
    return get_epoch_time_us() * 1000;
}