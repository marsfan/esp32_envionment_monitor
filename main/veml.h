/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// VEML7700 implementation.

// Developed from the following documents:
// https://www.vishay.com/docs/84286/veml7700.pdf
// https://cdn.sparkfun.com/assets/8/7/4/4/1/VEML7700_Application_Note.pdf

// Additionally, used the Sparkfun VEML7700 library for reference, which is MIT
// licensed.
// https://github.com/sparkfun/SparkFun_VEML7700_Arduino_Library

#ifndef VEML_H
#define VEML_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c.h"

#define VEML_ADDR 0x10  ///< I2C Address of the VEML 7700
#define I2C_MASTER_TIMEOUT_MS \
    1000 / portTICK_PERIOD_MS  ///< How long to wait before I2C transmission
                               ///< considered failed
#define VEML_REG_BYTES 2       ///< Size of a single register on the VEML

#define VEML_CONFIG_REG 0x00          ///< Configuration register address
#define VEML_THRESHOLD_HIGH_REG 0x01  ///< High threshold window setting
#define VEML_THRESHOLD_LOW_REG 0x02   ///< Low threshold window setting
#define VEML_POWER_SAVING_REG 0x03    ///< Power saving settings
#define VEML_ALS_LEVEL_REG 0x04       ///< ALS Level output register
#define VEML_WHITE_LEVEL_REG 0x05     ///< White level output register
#define VEML_INTERRUPT_REG 0x06       ///< Interrupt status register

// Enumerations of different configuration options

/// @brief Possible gain values for the sensor.
typedef enum {
    VEML_GAIN_1 = 0x0,     ///< 1x gain
    VEML_GAIN_2,           ///< 2x gain
    VEML_GAIN_1_8,         ///< 1/8x gain
    VEML_GAIN_1_4,         ///< 1/4x gain
    VEML_NUM_GAIN_OPTIONS  ///< Total number of gain options
} veml_gain_options_e;

/// @brief Possible integration times for the sensor.
typedef enum {
    VEML_INTEGRATION_25 = 0b1100,     ///< 25ms integration time
    VEML_INTEGRATION_50 = 0b1000,     ///< 50ms integration time
    VEML_INTEGRATION_100 = 0b0000,    ///< 100ms integration time
    VEML_INTEGRATION_200 = 0b0001,    ///< 200ms integration time
    VEML_INTEGRATION_400 = 0b0010,    ///< 400ms integration time
    VEML_INTEGRATION_800 = 0b0011,    ///< 800ms integration time
    VEML_NUM_INTEGRATION_OPTIONS = 6  ///< Total number of integration options
} veml_integration_options_e;

/// @brief Possible persistence protection number.
typedef enum {
    VEML_PERSISTENCE_1 = 0x0,     ///< Persistence Protect 1
    VEML_PERSISTENCE_2,           ///< Persistence Protect 2
    VEML_PERSISTENCE_4,           ///< Persistence Protect 4
    VEML_PERSISTENCE_8,           ///< Persistence Protect 8
    VEML_NUM_PERSISTENCE_OPTIONS  ///< Total number of persistence protect
                                  ///< options
} veml_persistence_protect_options_e;

/// @brief Enable/disable interrupt
typedef enum {
    VEML_INTERRUPT_DISABLE = 0x0,  ///< Disable ALS interrupt
    VEML_INTERRUPT_ENABLE,         ///< Enable ALS Interrupt
} veml_interrupt_options_e;

/// @brief Enable/disable sensor
typedef enum {
    VEML_POWER_ON = 0x00,  ///< Turn on sensor
    VEML_POWER_OFF,        ///< Turn off sensor
} veml_power_options_e;
// Definitions for registers that have bitwise configuration.

/// @brief The configuration register (address 0x00)
typedef struct {
    uint16_t reserved1         : 3;  ///< Reserved bits
    uint16_t gain              : 2;  ///< ALS Channel gain
    uint16_t reserved2         : 1;  ///< Reserved bits
    uint16_t integration_time  : 4;  ///< ALS channel integration time
    uint16_t persistence       : 2;  ///< ALS channel persistence protection
    uint16_t reserved3         : 2;  ///< Reserved bits
    uint16_t interrupt_enabled : 1;  ///< Whether or not to enable ALS interrupt
    uint16_t shutdown          : 1;  ///< Whether or not to shut down the ALS
} veml_config_reg_t;

/// @brief Power Saving Configuration register
typedef struct {
    uint16_t reserved            : 13;  ///< Reserved bits
    uint16_t power_saving_mode   : 2;   ///< Power Saving Mode
    uint16_t power_saving_enable : 1;   ///< Enable power saving
} veml_power_saving_reg_t;

/// @brief  VEML7700 Ambient Light Sensor interaction structure.
class Veml7700 {
   public:
    /*!
     * @brief Instantiate the device.
     * @param[in] i2c_port The i2c port to use for communicating with the sensor
     */
    Veml7700(i2c_port_t i2c_port);

    /*!
     *  @brief Set the sensor configuration over I2C
     *  @return Result of setting configuration.
     */
    esp_err_t set_configuration(void);

    /*!
     *  @brief Set the sensor configuration over I2C
     *  @param[in] config Pointer to the configuration to send to the sensor
     *  @return Result of setting configuration.
     */
    esp_err_t set_configuration(const veml_config_reg_t *const config);

    /*!
     * @brief Get the sensor configuration over I2C, store it in the instance.
     * @return Result of getting configuration.
     */
    esp_err_t get_configuration(void);

    /*!
     * @brief Get the sensor configuration over I2C, store it in the instance.
     * @param[out] config pointer to a struct to store the read config data
     * @return Result of getting configuration.
     */
    esp_err_t get_configuration(veml_config_reg_t *config);

    /*!
     * @brief Get the raw ambient light level from the sensor
     * @return The raw ambient light level from the sensor.
     */
    uint16_t get_ambient_level(void);

    /*!
     *  @brief Get the raw white light level from the sensor
     *  @return The raw white light level from the sensor.
     */
    uint16_t get_white_level(void);

    /*!
     *  @brief Get the computed ALS Lux value.
     *  @return The computed brightness in Lux
     */
    float get_lux(void);

    /*!
     *  @brief Get the current sensor gain
     *  @return The current sensor gain
     */
    veml_gain_options_e get_gain(void);

    /*!
     * @brief Set the sensor gain
     * @param[in] gain The gain to set the sensor
     * @return Result of getting configuration.
     */
    esp_err_t set_gain(const veml_gain_options_e gain);

    /*!
     * @brief Get the current integration time
     * @returns The current integration time
     */
    veml_integration_options_e get_integration_time(void);

    /*!
     * @brief Set the sensor integration time
     * @param[in] integration_time The integration time to set into the
     * sensor.
     * @return Result of getting configuration.
     */
    esp_err_t set_integration_time(
        const veml_integration_options_e integration_time);

    /*!
     * @brief Get whether or not the low-power mode is enabled
     * @returns Sensor power state
     */
    veml_power_options_e get_power_state(void);

    /*!
     * @brief Set the power state
     * @param[in] state The power state to set.
     * @return Result of getting configuration.
     */
    esp_err_t set_power_state(const veml_power_options_e state);

   private:
    /// @brief Sensor configuration
    veml_config_reg_t configuration;

    /// @brief  I2C port to use
    i2c_port_t i2c_port;

    /*!
     * @brief Write to the specific register
     * @param[in] reg The register to write to
     * @param[in] data The data to write to the register
     * @returns Error code result from the I2C operation.
     */
    esp_err_t write_to_reg(const uint8_t reg, const uint16_t *const data);

    /*!
     * @brief Read from the specific register
     * @param[in] reg The register to write to
     * @param[out] data The data read from the register
     * @returns Error code from the I2C Operation
     */
    esp_err_t read_from_reg(const uint8_t reg, uint16_t *const data);

    /*!
     * @brief Get the scaling factor to convert raw ALS value to Lux.
     * @return Scaling factor to convert raw ALS value to Lux.
     */
    float get_als_scale(void);
};

#endif  // VEML_H