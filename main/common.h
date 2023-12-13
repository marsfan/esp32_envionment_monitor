/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief Common functionality to use across modules.

#ifndef COMMON_H
#define COMMON_H

#include <esp_err.h>

/*!
 * @brief Print to log, but only on an error.
 * @param[in] err: The error value. Loggin only occurs if this is not ESP_OK
 * @param[in] tag Error tag to show in log message
 * @param[in] func The function the error ocurred in.
 * @param[in] msg Error message to display
 */
void log_e_on_error(esp_err_t err, const char *const tag,
                    const char *const func, const char *const msg);

#endif  // COMMON_H