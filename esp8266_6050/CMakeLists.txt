cmake_minimum_required(VERSION 2.8.4)
set(ARDUINO_SDK_PATH /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arduino-cmake/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME esp8266_6050)
set(TEST rebuild)
project(${PROJECT_NAME})

set(${CMAKE_PROJECT_NAME}_SKETCH esp8266_6050.ino)

#### Uncomment below additional settings as needed.
# set(${CMAKE_PROJECT_NAME}_BOARD mega)
# set(${CMAKE_PROJECT_NAME}_PORT /dev/ttyACM0)
# set(mega.build.mcu atmega2560)
# set(mega.upload.protocol wiring)
# set(mega.upload.speed 115200)

generate_arduino_firmware(${CMAKE_PROJECT_NAME})
