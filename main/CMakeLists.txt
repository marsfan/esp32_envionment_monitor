#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https: //mozilla.org/MPL/2.0/.
#
idf_component_register(SRCS "bme688.cpp" "common.cpp" "veml.cpp" "main.cpp" "bme68x_sensor_api/bme68x.c"
                    INCLUDE_DIRS ".")
