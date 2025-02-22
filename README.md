# ESP32 Sensor Firmware

## Overview
This project is an ESP32 firmware that interfaces with an SHT4X temperature and humidity sensor over I2C. The firmware initializes the I2C bus, reads sensor data periodically, and logs the results. It uses FreeRTOS tasks and mutexes to ensure thread-safe access to the sensor.

## Features
- Communicates with the SHT4X sensor via I2C
- Performs temperature and humidity measurements at regular intervals
- Implements CRC checking for data integrity
- Logs sensor readings to the console
- Uses FreeRTOS for task scheduling
- Includes a retry mechanism for I2C initialization
- Provides a GitHub Actions CI pipeline for automated builds

## Hardware Requirements
- ESP32 development board
- SHT4X temperature and humidity sensor
- I2C pull-up resistors (if not included on the sensor module)

## Software Requirements
- ESP-IDF (Espressif IoT Development Framework)
- Python 3.x (for ESP-IDF setup)
- Git

## Setup & Build Instructions
### 1. Install ESP-IDF
```sh
git clone --recursive https://github.com/espressif/esp-idf.git esp-idf
cd esp-idf
./install.sh
. ./export.sh
```

### 2. Clone this repository
```sh
git clone https://github.com/rnicolas/currentt_assignment.git
cd currentt_assignment
```

### 3. Build the firmware
```sh
. esp-idf/export.sh
idf.py build
```

### 4. Flash the firmware
```sh
idf.py flash
```

### 5. Monitor serial output
```sh
idf.py monitor
```

## Code Structure
```
├── main/
│   ├── app_main.c              # Application entry point
│   ├── sht4x_i2c_driver.c      # Driver for SHT4X sensor
│   ├── sht4x_i2c_driver.h      # Header file for SHT4X driver
├── CMakeLists.txt              # CMake build system
├── sdkconfig                   # ESP-IDF configuration file
├── .github/workflows/
│   ├── build.yml               # GitHub Actions workflow
└── README.md                   # Project documentation
```

## GitHub Actions CI/CD
This project includes a GitHub Actions workflow to automatically build the firmware on every push and pull request. The workflow:
1. Sets up the build environment
2. Installs dependencies
3. Builds the firmware
4. Uploads the firmware artifacts

## Author
**Roger N. Alegret**

## Contact
For questions or contributions, feel free to open an issue or create a pull request!