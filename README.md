# BNO055 External Component for ESPHome

This is an external component for ESPHome that provides support for the BNO055 9-DOF sensor.

## Features

- Accelerometer (X, Y, Z)
- Gyroscope (X, Y, Z)
- Magnetometer (X, Y, Z)
- Euler angles (Heading, Roll, Pitch)
- Linear acceleration (X, Y, Z)
- Gravity (X, Y, Z)
- Temperature
- Compass heading (Magnetic North) with automatic tilt compensation

## Installation

1. Add this repository to your ESPHome configuration
2. Include the component in your YAML configuration
3. Configure I2C pins and sensor addresses

## Usage

See `bno055.yaml` for a complete example configuration.

## Marine Applications

This component is optimized for marine use with automatic tilt compensation. The BNO055's NDOF mode provides accurate compass readings even when the device is tilted due to boat movement (roll and pitch).

## Wiring

- VCC to 3.3V
- GND to GND
- SDA to GPIO 21 (configurable)
- SCL to GPIO 22 (configurable)

## License

MIT License 