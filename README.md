# Ultrasonic Distance Measurement with Temperature Compensation and Servo Motor Control

This project involves measuring distance using an ultrasonic sensor and compensating for temperature variations using an I2C temperature sensor on an embedded platform. Additionally, it controls a servo motor based on the measured distance. Distance and temperature data are transmitted via UART to a PC for monitoring and analysis.

## Description

The system calculates distance by measuring the time taken for an ultrasonic sound pulse to travel to an object and back. It compensates for temperature variations to adjust the speed of sound used in distance calculations, based on real-time temperature data from the I2C temperature sensor. The measured distance is then used to generate a PWM signal that controls the servo motor. Based on the distance, the servo motor is turned to a specific angle.

### Components

- **Ultrasonic Sensor**: Measures distance based on the time delay of sound waves.
- **I2C Temperature Sensor**: Provides temperature data for calculating the speed of sound.
- **Microcontroller (STM32F746ZG)**: Controls sensor operations, calculates distances, manages temperature compensation, handles timers for precise timing, and controls the servo motor.
- **UART Interface**: Sends distance, temperature, and servo angle data to a PC for monitoring and analysis.
- **Timers**: Used to measure the time taken for ultrasonic sound waves to return to the sensor, to generate PWM signal for Servo motor control and for various other purposes.
- **Servo Motor**: Adjusts its angle based on the measured distance using PWM control.
