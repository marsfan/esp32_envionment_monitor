/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief Base class for creating a class that uses a single mutex.
/// @details Other classses are subclassed from this to avoid repetition.
#ifndef MUTEX_BASE_H
#define MUTEX_BASE_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class MutexBase {
   protected:
    /// @brief instantiate the class.
    /// @details When instantiated this way, the wait time will be the
    /// MAX_MUTEX_WAIT_TICKS defined in "common.h"
    MutexBase(void);

    /// @brief Instantiate the class and specify the max allowed wait time to
    /// obtain the mutex.
    /// @param wait_time The maximum allowed amount of time to wait for the
    /// mutex.
    MutexBase(const TickType_t wait_time);

    /// @brief Take the classes's mutex.
    /// @param tag Tag to use when logging a failed attempt at reading the
    /// mutex.
    /// @param func The name of the calling function.
    /// @return Boolean indicting if the mutex was successfully taken.
    bool take_mutex(const char *const tag, const char *const func);

    /// @brief Release classes's mutex.
    /// @param tag Tag to use when logging a failed attempt at reading the
    /// mutex.
    /// @param func The name of the calling function.'
    void release_mutex(const char *const tag, const char *const func);

   private:
    /// @brief  Mutex to ensure task safety.
    SemaphoreHandle_t mutex;

    /// @brief Time to wait to obtain the mutex.
    TickType_t wait_time;
};
#endif  // MUTEX_BASE_H