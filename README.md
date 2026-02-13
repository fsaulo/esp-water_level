ESP RainMaker Water Level Monitor
=================================

Description
-----------

This project implements a residential water level monitoring system using ESP RainMaker on the ESP32 platform. The system measures the water level of a storage tank using a JSN-SR04T ultrasonic sensor and reports the data to the cloud via ESP RainMaker.

The firmware is designed for the ESP32 and has been tested on the ESP32-WROOM-32 DevKit v4 development board.

The system enables remote monitoring of tank level percentage, distance measurement, and device status through the ESP RainMaker mobile application.

Hardware
--------

Target MCU:
- ESP32

Tested Board:
- ESP32-WROOM-32 DevKit v4

Sensor:
- JSN-SR04T waterproof ultrasonic distance sensor

Note:
Ensure proper voltage compatibility between the JSN-SR04T echo pin and the ESP32 (3.3V logic).

Features
--------

- Ultrasonic distance measurement
- Water level calculation based on tank height configuration
- Percentage level reporting (0–100%)
- Cloud connectivity via ESP RainMaker
- OTA support
- NVS storage for configuration parameters
- [Secure provisioning](https://docs.rainmaker.espressif.com/docs/dev/dashboard-cli/rainmaker-cli-provisioning/#usage-guide)

Software Requirements
---------------------

- ESP-IDF (version compatible with the ESP RainMaker release used)
- ESP RainMaker component
- Python (for ESP-IDF tools)

Configuration
-------------

Before building, configure the project:

    idf.py menuconfig

Important configuration sections:

- Wi-Fi credentials (if static provisioning is used)
- RainMaker configuration
- Partition table (custom layout recommended for larger binaries)
- Flash size (must match hardware)
- Security features (if flash encryption or secure boot is enabled)

Tank parameters such as height, minimum level, and calibration factors should be configured in firmware or stored in NVS depending on implementation.

Build and Flash
---------------

Build the project:

    idf.py build

Flash the firmware:

    idf.py -p <PORT> flash monitor

Provision the device using the ESP RainMaker mobile application.

Operation
---------

The device periodically measures the distance between the sensor and the water surface. Based on configured tank height and calibration parameters, it computes:

- Current water height
- Fill percentage
- Empty/full status

This data is published to ESP RainMaker and can be monitored remotely.

Notes
-----

- Ensure the ultrasonic sensor is mounted perpendicular to the water surface.
- Avoid condensation or obstructions inside the tank.
- Large firmware sizes require appropriately sized OTA partitions.
