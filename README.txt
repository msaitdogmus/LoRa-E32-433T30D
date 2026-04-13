# LoRa-E32-433T30D

This repository contains a simple LoRa communication project using the E32-433T30D module.

There are two main files in this project:
-> "transmitter.ino"
-> "receiver.ino"

These files are used for sending and receiving data between two devices.

The transmitter side collects sensor data and sends it with the LoRa module.  
The receiver side reads this data and prints it to the serial monitor.

You can also change and adjust the code depending on the sensors you use.

I used this project for Teknofest, and it can also be used for basic LoRa communication tests.

This project was developed by a single person.

## Libraries

This project uses the following libraries:

- `LoRa_E32`
- `TinyGPS++`
- `Adafruit_BMP3XX`
- `Adafruit_Sensor`
- `SoftwareSerial`
- `Adafruit_MPU6050`

## Hardware / Sensor Usage

In this project:

- `BMP388` is used for pressure, altitude, and temperature data
- `MPU6050` is used for acceleration and gyroscope data
- `GPS` is used for latitude and longitude information
- `LoRa E32-433T30D` is used for wireless data transmission

## How It Works

The transmitter reads data from the sensors and GPS module.  
After that, it puts all values into a data structure and sends them to the receiver with LoRa.

The receiver listens for incoming data, reads the packet, and shows the values on the serial monitor.

The transmitted data can include:

- pressure
- altitude
- temperature
- acceleration values
- gyroscope values
- GPS latitude and longitude

## Configuration

The project already has its own internal configuration settings in the code.

Address, channel, baud rate, transmission mode, and power settings are configured directly inside the Arduino code.

Because of this, there is no need to use an extra program to configure the transmitter or receiver module.

You can change these settings manually in the code if needed.

## Notes

- Make sure the same channel settings are used on both sides
- Addresses should be set correctly for transmitter and receiver communication
- You can modify the project for different sensors and different test scenarios
- This project is suitable for simple LoRa experiments, telemetry tests, and student projects
