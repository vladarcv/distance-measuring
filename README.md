# Ultrasonic Distance Measurement with Temperature Compensation

This project involves measuring distance using an ultrasonic sensor and compensating for temperature variations using an I2C temperature sensor on an embedded platform. Distance and temperature data are transmitted via UART to a PC for monitoring and analysis.

## Description

The system calculates distance by measuring the time taken for an ultrasonic sound pulse to travel to an object and back. It compensates for temperature variations to adjust the speed of sound used in distance calculations, based on real-time temperature data from the I2C temperature sensor.

### Components:

- **Ultrasonic Sensor**: Measures distance based on time delay of sound waves.
- **I2C Temperature Sensor**: Provides temperature data for calculating the speed of sound.
- **Microcontroller (STM32F746ZG)**: Controls sensor operations, calculates distances, manages temperature compensation, and handles timers for precise timing.
- **UART Interface**: Sends distance and temperature data to a PC for monitoring and analysis.
- **Timers**: Used to measure the time taken for ultrasonic sound waves to return to the sensor, and for various other purposes.
