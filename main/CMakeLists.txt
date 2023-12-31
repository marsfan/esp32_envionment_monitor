#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https: //mozilla.org/MPL/2.0/.
#
idf_component_register(SRCS "mutex_base.cpp" "sensor_hub.cpp" "safe_i2c.cpp" "mqtt_task.cpp" "wifi_network.cpp" "bme688.cpp" "common.cpp" "veml.cpp" "main.cpp" "bme68x_sensor_api/bme68x.c" "bsec.cpp"
                    INCLUDE_DIRS ".")

# Add the BSEC library to be linked in
# TODO: Conditional based on ESP model?
add_prebuilt_library(bsec "bsec/bin/esp/esp32/libalgobsec.a")
# Main uses functions from bsec, so link them
target_link_libraries(${COMPONENT_LIB} PRIVATE bsec)
