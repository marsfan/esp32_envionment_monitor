# Environment Monitoring Program

This iss my first attempt at a simple household environment monitoring system
that uses an ESP-32, a BME688, and a VEML7700.

## Extra Files

There are a few files not included in this repository that are necessary to
compile the project. The Bosch BSEC library, and a WiFi configuration file.

### BSEC Library

I have not included the BSEC library in the repo, as I am not sure about the
licensing of doing that. You can get the library for free and add the missing
files. See the file [placeholder.md](main/bsec/placeholder.md) for instructions

### WiFi and MQTT Configuration

Also not included is the file `private_config.h`. There is not much to
this file, it just defines certain options that I want to keep private.
Simple create the file with the following preprocessor definitions

* `WIFI_SSID`: String of the SSID of the network to connect to
* `WIFI_PASSWORD`: String of the password for connecting to WiFi
* `MQTT_BROKER_URI`: The URI to publish MQTT data to.
* `MQTT_USERNAME`: The username to use for logging into the MQTT broker
* `MQTT_PASSWORD`: The password to use for logging into the MQTT broker
* `MQTT_TEMP_TOPIC`: The MQTT Topic to use for publishing the temperature.

More fields will be added to this over time

## Cable Color Mapping

I am using my ribbon cable adapter from the Bus Pirate with the ESP-PROG
Here is the color mapping

| Color  | JTAG Purpose | SPI Purpose | I2C Purpose | UART Purpose |
| ------ | ------------ | ----------- | ----------- | ------------ |
| Brown  | VCC          | VCC         | VCC         | VCC          |
| Red    | TMS          | CS          |             | CTS          |
| Orange | GND          | GND         | GND         | GND          |
| Yellow | TCK          | SCK         | SCL         | TXD          |
| Green  | GND          | GND         | GND         | GND          |
| Blue   | TDO          | MISO/DI     | SDA         | RTS          |
| Purple | GND          | GND         | GND         | GND          |
| Gray   | TDI          | MOSI/DO     | SDA         | RXD          |
| White  | GND          | GND         | GND         | GND          |
| Black  | NC           | NC          | NC          | NC           |

_Note: Blue and Gray must be tied together in I2C Mode_
