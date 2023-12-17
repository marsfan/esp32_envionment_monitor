# Environment Monitoring Program

This iss my first attempt at a simple household environment monitoring system
that uses an ESP-32, a BME688, and a VEML7700.

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
